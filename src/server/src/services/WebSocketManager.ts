import { WebSocketServer, WebSocket } from 'ws';
import { PORTS } from '../config';
import { Sequelize } from 'sequelize';

export class WebSocketManager {
    private incomingServer: WebSocketServer;
    private outgoingServer: WebSocketServer;
    private clients: Set<WebSocket> = new Set();
    
    static sequelize = new Sequelize('postgres://user:password@localhost:5432/drones');

    constructor() {
        // Инициализация входящего сервера (для ROS)
        this.incomingServer = new WebSocketServer({ port: PORTS.INCOMING });
        
        // Инициализация исходящего сервера (для Foxglove)
        this.outgoingServer = new WebSocketServer({ port: PORTS.OUTGOING });
        
        this.setupIncoming();
        this.setupOutgoing();
        
        this.initializeDatabase();
    }

    private initializeDatabase() {
        WebSocketManager.sequelize.authenticate()
            .then(() => console.log('Database connected'))
            .catch(error => console.error('Database connection error:', error));
    }

    private setupIncoming() {
        this.incomingServer.on('connection', (ws) => {
            console.log('ROS client connected');
            
            ws.on('message', (data) => {
                try {
                    const message = JSON.parse(data.toString());
                    this.processIncoming(message);
                } catch (error) {
                    console.error('Invalid message format:', error);
                }
            });
        });
    }

    private setupOutgoing() {
        this.outgoingServer.on('connection', (ws) => {
            console.log('Foxglove client connected');
            this.clients.add(ws);
            
            ws.on('close', () => {
                this.clients.delete(ws);
            });
        });
    }

    private async processIncoming(data: any) {
        // Здесь можно добавить дополнительную логику обработки
        console.log('Received from ROS:', data);
    }

    public broadcast(data: any) {
        const message = JSON.stringify(data);
        this.clients.forEach(client => {
            if (client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        });
    }
}