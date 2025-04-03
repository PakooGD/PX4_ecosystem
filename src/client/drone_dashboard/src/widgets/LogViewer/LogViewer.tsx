import React, { useEffect, useRef, useState } from 'react';
import { observer } from 'mobx-react-lite';
import { DroneTopicList} from '../../features/droneLog/ui/DroneTopicList';
import { droneStore } from '../../shared/stores/droneStore';
import { v4 as uuidv4 } from 'uuid';
import { Drone, LogMessage } from '../../shared/types/ITypes';

export const LogViewer: React.FC = observer(() => {
    
    const socketRef = useRef<WebSocket | null>(null);
    const [isConnected, setIsConnected] = useState(false);
    const [connectionError, setConnectionError] = useState<string | null>(null);
    const reactId = uuidv4();

    useEffect(() => {
        let reconnectAttempts = 0;
        const maxReconnectAttempts = 5;
        const connect = () => {
            const socket = new WebSocket('ws://localhost:8083');
            socketRef.current = socket;
    
            socket.onopen = () => {
                console.log('WebSocket connection opened');
                setIsConnected(true);
                setConnectionError(null);
                socket.send(JSON.stringify({ type: 'register', id: reactId }));
                reconnectAttempts = 0; 
            };
    
            socket.onmessage = (event) => {
                const { source, content } = JSON.parse(event.data);
                const result: LogMessage = {
                    droneId: source,
                    content: content,
                };
                console.log()
                droneStore.addLog(result);
            };
    
            socket.onclose = () => {
                console.log('WebSocket connection closed');
                setIsConnected(false);
                if (reconnectAttempts < maxReconnectAttempts) {
                    reconnectAttempts++;
                    console.log(`Reconnecting... Attempt ${reconnectAttempts}`);
                    setTimeout(connect, 5000); // Пытаемся переподключиться через 5 секунд
                } else {
                    setConnectionError('Failed to reconnect. Please check your internet connection.');
                }
            };
    
            socket.onerror = (error) => {
                console.error('WebSocket error:', error);
                setIsConnected(false);
                setConnectionError('WebSocket error. Please try again later.');
                socket.close(); // Закрываем соединение при ошибке
            };
        };
    
        connect(); // Первоначальное подключение
    
        return () => {
            if (socketRef.current) {
                socketRef.current.close();
                socketRef.current = null;
            }
        };
    }, []);

    const droneIds: Drone[] = droneStore.drones;
    
    return (
        <div>
            <h1>Logs</h1>
            {!isConnected && connectionError && (
                <div style={{ color: 'red' }}>{connectionError}</div>
            )}
            {droneIds.map((drone) => {
                const ulog = droneStore.ulog.get(drone.id)
                const topics = droneStore.logs.get(drone.id);
                if (topics || ulog) {
                    return (
                        <DroneTopicList
                            key={`${drone.id}_topics`} 
                            droneId={drone.id}
                            topics={topics}
                            ulog={ulog}
                        />
                    );
                }
                return null;
            })}
        </div>
    );
});      
