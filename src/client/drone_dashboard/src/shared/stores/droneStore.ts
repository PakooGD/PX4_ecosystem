// src/shared/stores/droneStore.ts
import { makeAutoObservable, observable } from 'mobx';
import { fetchDrones, updateTopics, redirectLogs, loadLogs } from '../api/api';
import { DroneData,Log,TopicStatus, LogMessage, ULogData, TopicData, TopicSchema } from '../types/ITypes';
import { v4 as uuidv4 } from 'uuid';
import { MessageType, ULog } from "@foxglove/ulog";

function formatString(input: string) {
    return input
        .split('_')
        .map(word => word.charAt(0).toUpperCase() + word.slice(1))
        .join('');
}

class DroneStore {
    drones: DroneData[] = [];
    selectedDroneId: string | null = null;
    socket: WebSocket | null = null
    logs = observable.map<string, Map<string, Log[]>>(); // droneId -> topic -> logs
    // ulog: any[] = [];
    ulog: Map<string, ULogData> = new Map();

    constructor() {
        makeAutoObservable(this);
        this.loadDrones();
    }

    async loadDrones() {
        try {
            const result = await fetchDrones();
            this.drones = result.map(drone => ({
                id: drone.id,
                topics: drone.topics,
                status: drone.status,
                ip_address: drone.ip_address,
            }));
        } catch (error) {
            console.error('Failed to load drones:', error);
        }
    }

    async updateSubscriptions(droneId: string, topics: TopicStatus[]) {
        try {
            await updateTopics({ drone_id: droneId, topics: topics });
            console.log('Subscriptions updated successfully for drone:', droneId);
        } catch (error) {
            console.error('Failed to update subscriptions:', error);
            throw error;
        }
    }

    async redirectLogs(droneId: string, selectedPath: string) {
        try {
            await redirectLogs({ droneId: droneId, selectedPath: selectedPath });
            this.selectedDroneId = droneId
            console.log('LogDirection updated successfully for drone:', droneId);
        } catch (error) {
            console.error('LogDirection to update subscriptions:', error);
            throw error;
        }
    }

    async loadLogs(droneId: string) {
        try {
            const {filelike, filename} = await loadLogs({ droneId });

            // Чтение и обработка файла
            const ulog = new ULog(filelike);
            await ulog.open();
            
            const topics = new Map<string, any[]>();
            
            for await (const msg of ulog.readMessages()) {
                if (msg.type === MessageType.Data) {
                    const dataTopic = String(ulog.subscriptions.get(msg.msgId)?.name);
                    const name = formatString(dataTopic);

                    const message: TopicData = {
                        name: name,
                        topic: dataTopic,
                        timestamp: BigInt(msg.value.timestamp),
                        data: msg.value,
                    };

                    // Если топик еще не существует в Map, создаем для него пустой массив
                    if (!topics.has(dataTopic)) {
                        topics.set(dataTopic, []);
                    }

                    // Добавляем сообщение в массив для соответствующего топика
                    topics.get(dataTopic)!.push(message);
                }
            }
  
            const result = {
                source: filename,
                content: Object.fromEntries(topics),
            };
            
            this.ulog.set(droneId,result)

        } catch (error) {
            throw error;
        }
    }

    addLog(data: LogMessage) {
        const log: Log = {
            id: uuidv4(),
            timestamp: new Date(Number(data.content.timestamp)).toISOString(),
            data: data.content,
        };

        if (!this.logs.has(data.droneId)) {
            this.logs.set(data.droneId, observable.map<string, Log[]>());
        }
        const droneLogs = this.logs.get(data.droneId)!;

        if (!droneLogs.has(data.content.topic)) {
            droneLogs.set(data.content.topic, observable.array<Log>());
        }
        const topicLogs = droneLogs.get(data.content.topic)!;

        topicLogs.push(log);
    }

    getLogAtTime(droneId: string, topic: string, timestamp: string): Log | undefined {
        const topicLogs = this.logs.get(droneId)?.get(topic);
        if (!topicLogs) return undefined;

        return topicLogs.find((log) => log.timestamp <= timestamp);
    }
}

export const droneStore = new DroneStore();
