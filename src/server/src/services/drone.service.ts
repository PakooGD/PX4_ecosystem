import { TopicStatus, DataVisualizer } from '../types/ITypes';
import { FoxgloveVisualizer, ReactVisualizer } from '../utils/LogVisualizers';
import { OperationFailed } from '../utils/errors/errors';
import { droneClients,server } from '../app';
import { Channels, Subscriptions } from '../utils/LogVisualizers/FoxgloveVisualizer';
import { AuthService } from './auth.service';
import fs from 'fs';
import path from 'path';

const visualizers = new Map<string, DataVisualizer>();

export class DroneHandler {

    public static RedirectLogs(data: any): void {
        try {
            let visualizer: DataVisualizer = new ReactVisualizer();;
            switch (data.selectedPath) {
                case 'site':
                    visualizer = new ReactVisualizer();
                    break;
                case 'foxglove':
                    visualizer = new FoxgloveVisualizer();
                    break;
                default:
                    throw new Error(`Unknown visualizer path: ${data.selectedPath}`);
            }
            visualizers.set(data.droneId, visualizer);
        } catch (err) {
            throw new OperationFailed(`Failed to set visualizer: ${err}`);
        }
    }

    public static async loadLogs(data:any, res: any): Promise<any> {
        try {
            // Находим все файлы, начинающиеся на drone_id и заканчивающиеся на .ulog
            const { droneId } = data

            const logDir = '/home/alexei/Projects/PX4_ecosystem/src/server/temp/ulog';

            if (!fs.existsSync(logDir)) {
                throw new OperationFailed(`Log directory not found: ${logDir}`);
            }

            const files = fs.readdirSync(logDir)
                .filter(file => file.startsWith(droneId) && file.endsWith('.ulg'))
                .map(file => ({
                    name: file,
                    path: path.join(logDir, file),
                    time: fs.statSync(path.join(logDir, file)).mtime.getTime() // Время последнего изменения
                }));

            if (files.length === 0) {
                throw new OperationFailed(`No log files found for drone ${droneId}`);
            }

            // Выбираем файл с самой последней датой
            const latestFile = files.reduce((prev, current) => 
                (prev.time > current.time) ? prev : current
            );
            const fileData = fs.readFileSync(latestFile.path);
            
        // Устанавливаем заголовки для скачивания файла
        res.setHeader('Content-Type', 'application/octet-stream');
        res.setHeader('Content-Disposition', `attachment; filename="${latestFile.name}"`);
        res.send(fileData);
        } catch (err) {
            throw new OperationFailed('Failed to load file');
        }
    }

    public static async HandleData(droneId: string, data: any): Promise<void> {
        try {
            const visualizer = visualizers.get(droneId);
            if (visualizer) {
                await visualizer.visualizeData(droneId, data);
            } else {
                visualizers.set(data.droneId, new ReactVisualizer());
            }
        } catch (err) {
            throw new OperationFailed('Failed to send data');
        }
    }

    public static async HandleTopics(data: any): Promise<void> {
        try {
            const connection = droneClients.get(data.drone_id);
            const topicsToProceed: TopicStatus[] = data.topics;
            topicsToProceed.forEach(item => {
                const keyName = `${item.topic} [${data.drone_id}]`
                if (item.status) {
                    connection?.send(JSON.stringify({ type: 'subscribe', topic: item.topic }));
                } else {
                    connection?.send(JSON.stringify({ type: 'unsubscribe', topic: item.topic }));
                    if (Channels.has(keyName)) {
                        server?.removeChannel(Channels.get(keyName)!);
                        Channels.delete(keyName)
                    }
                }
                AuthService.UpdateData(data.drone_id, data)
                Subscriptions.set(keyName, item.status)
            });
        } catch (err) {
            throw new OperationFailed('Failed');
        }
    }
}