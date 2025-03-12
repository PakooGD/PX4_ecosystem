import express from 'express';
import { WebSocketManager } from './utils/websocket';
import { router } from './routes';

const app = express();
const PORT = 3000;
const WS_PORT = 8081;

// Middleware
app.use(express.json());

// Роуты
app.use('/api', router);

// WebSocket сервер
const wsManager = new WebSocketManager(WS_PORT);

// Запуск HTTP сервера
app.listen(PORT, () => {
  console.log(`HTTP server running on port ${PORT}`);
});

console.log(`WebSocket server running on ws://localhost:${WS_PORT}`);