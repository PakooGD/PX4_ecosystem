import { WebSocketServer } from 'ws';
import { FoxgloveServer } from '@foxglove/ws-protocol';

const FoxgloveStudioPort = 8081;
const BridgePort = 8082;

interface TopicData {
    name: string;
    topic: string;
    timestamp: number;
    data: Record<string, any>;
}

const server = new FoxgloveServer({ name: "px4-foxglove-bridge" });

const ws = new WebSocketServer({
    port: FoxgloveStudioPort,
    handleProtocols: (protocols) => server.handleProtocols(protocols),
});

const customChannels = new Map<string, number>();

ws.on("connection", (conn, req) => {
    console.log('🎮 Foxglove Studio connected');
    const name = `${req.socket.remoteAddress}:${req.socket.remotePort}`;

    const wss = new WebSocketServer({ port: BridgePort });

    wss.on('connection', (ws) => {
        ws.on('message', (message: string) => {
            const data: TopicData = JSON.parse(message);
            if (!customChannels.has(data.topic)) {
                // Генерация схемы на основе структуры данных
                const generateSchema = (obj: Record<string, any>): any => {
                    const schema: any = { type: "object", properties: {} };
                    for (const key in obj) {
                        if (typeof obj[key] === 'object' && !Array.isArray(obj[key])) {
                            // Рекурсивно генерируем схему для вложенных объектов
                            schema.properties[key] = generateSchema(obj[key]);
                        } else {
                            // Простые типы данных
                            schema.properties[key] = { type: typeof obj[key] === 'number' ? 'number' : 'object' };
                        }
                    }
                    return schema;
                };

                const schema = generateSchema(data.data);

                const channelId = server.addChannel({
                    topic: data.name,
                    encoding: "json",
                    schemaName: data.topic,
                    schema: JSON.stringify(schema),
                });

                // Сохраняем канал в нашем хранилище
                customChannels.set(data.topic, channelId);
            }

            // Отправка данных в Foxglove Studio
            const textEncoder = new TextEncoder();
            server.sendMessage(
                customChannels.get(data.topic)!,
                BigInt(data.timestamp),
                textEncoder.encode(JSON.stringify(data.data)),
            );
        });
    });

    server.handleConnection(conn, name);
});

console.log(`WebSocket server is running on ws://localhost:${BridgePort}`);
console.log(`Foxglove Studio server is running on ws://localhost:${FoxgloveStudioPort}`);