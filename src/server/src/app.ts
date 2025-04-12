import express from 'express';
import droneRoutes, {eventEmitter} from './routes/drone.routes';
import { EventTypes } from './types/ITypes';
import { handleErrors } from './middleware/error.middleware';
import WebSocket, { WebSocketServer } from 'ws';
import { FoxgloveServer } from '@foxglove/ws-protocol';
import cors from 'cors';
import { CryptoService, AuthService, LogService } from './services';
import sequelize from './config/database';
import { dronePort, foxglovePort, httpPort, reactPort } from './config/serverConfig'



require('dotenv').config();

const cryptoService = CryptoService.getInstance();

const foxgloveServer = new WebSocketServer({ port: foxglovePort, handleProtocols: (protocols: any) => server?.handleProtocols(protocols)!});
const reactServer = new WebSocketServer({ port: reactPort });
const droneServer = new WebSocketServer({ port: dronePort });

export const server = new FoxgloveServer({ name: "px4-foxglove-bridge" });

export const reactClients = new Map<string, WebSocket | null>();
export const droneClients = new Map<string, WebSocket | null>();

eventEmitter.emit(EventTypes.SET_OFFLINE_STATUS);

const app = express();
app.use(cors({
    origin: process.env.CORS_ORIGIN || 'http://localhost:3000',
    credentials: true,
    allowedHeaders: ['Content-Type', 'Authorization'],
}));
app.use(express.json());
app.use('/api', droneRoutes);
app.use(handleErrors);



sequelize.authenticate()
    .then(async () => {
        console.log('Connection to PostgreSQL has been established successfully.');
        
        // Sync models with database
        await sequelize.sync({ alter: true });
        console.log('Database models synchronized');
        
        // Start HTTP server
        app.listen(httpPort, () => {
            console.log(`HTTP server running on http://localhost:${httpPort}`);
            console.log(`Foxglove server running on ws://localhost:${foxglovePort}`);
            console.log(`React client server running on ws://localhost:${reactPort}`);
            console.log(`Drone server running on ws://localhost:${dronePort}`);
        });
    })
    .catch((error) => {
        console.error('Unable to connect to the database:', error);
        process.exit(1);
    });

reactServer.on('connection', (ws) => {
    ws.on('message', (message: string) => {
        try {
            const data = JSON.parse(message);
            if (data.type === 'register' && data.id) {
                reactClients.set(data.id, ws);
                console.log(`Client registered for drone ${data.id}`);
            }
        } catch (err) {
            console.error('Error processing message:', err);
        }
    });

    ws.on('close', () => {
        console.log('Client disconnected');
        for (const [id, client] of reactClients.entries()) {
            if (client === ws) {
                reactClients.delete(id);
                console.log(`Client unregistered for drone ${id}`);
            }
        }
    });
});

foxgloveServer.on("connection", (conn: any, req: any) => {
    const name = `${req.socket.remoteAddress}:${req.socket.remotePort}`;

    server.on("error", (err) => {
        console.error("server error: %o", err);
    });

    server?.handleConnection(conn, name);
});

droneServer.on('connection', async (ws, req) => {
    try {
        const decoded = await AuthService.verifyAuthTokens(req)
        const droneId = decoded.drone_id

        ws.send(JSON.stringify({
            type: 'keyExchange',
            publicKey: cryptoService.getPublicKey()
        }));

        eventEmitter.emit(EventTypes.SIGNIN, droneId);
        droneClients.set(droneId, ws);

        ws.on('message', async (message: string) => {
            try {
                const data = JSON.parse(message);

                if (data.type === 'session_init') {
                    cryptoService.HandleSessionInit(data, droneId, ws); 
                } else {
                    const decryptedData = cryptoService.decryptDroneMessage(droneId, data);

                    if (decryptedData.type === 'data') eventEmitter.emit(EventTypes.RECEIVED_DATA, droneId, decryptedData); 
                    if (decryptedData.type === 'info') eventEmitter.emit(EventTypes.UPDATE_DATA, droneId, decryptedData); 
                    if (decryptedData.type === 'ulog') eventEmitter.emit(EventTypes.STREAM_DATA, decryptedData, droneId);
                    if (decryptedData.type === 'file_chunk') eventEmitter.emit(EventTypes.RECIEVE_LOG, decryptedData);
    
                    ws.send(JSON.stringify({ type: 'ack' }));
                }
            } catch (err) {
                console.log(err)
            }
        });

        ws.on('close', async () => {
            try {
                await LogService.CloseStream()
            } catch (err) {
                console.error('Error during WebSocket close:', err);
            } finally {
                droneClients.delete(droneId);
                eventEmitter.emit(EventTypes.LOGOUT, droneId);
            }
        });      
    } catch(error) {
        console.error(error);
    }
});


// Cleanup on server shutdown
process.on('SIGINT', async () => {
    console.log('Shutting down servers...');
    
    try {
        eventEmitter.emit(EventTypes.SET_OFFLINE_STATUS);
        
        // Close all WebSocket connections
        reactServer.clients.forEach(client => client.close());
        foxgloveServer.clients.forEach(client => client.close());
        droneServer.clients.forEach(client => client.close());
        
        // Close servers
        reactServer.close();
        foxgloveServer.close();
        droneServer.close();
        
        // Close database connection
        await sequelize.close();
        
        process.exit(0);
    } catch (err) {
        console.error('Error during shutdown:', err);
        process.exit(1);
    }
});



