import express from 'express';
import { WebSocketServer } from 'ws';
import { router } from './routes';

const app = express();
const PORT = process.env.PORT || 3000;
const wss = new WebSocketServer({ port: 8081 });

// Middleware
app.use(express.json());

// Роуты
app.use('/api', router);

// WebSocket сервер
wss.on('connection', (ws) => {
  console.log('New client connected');

  ws.on('message', (message) => {
    const data = JSON.parse(message.toString());
    console.log(`Received: ${JSON.stringify(data, null, 2)}`);

    // Отправка данных в Foxglove Studio
    wss.clients.forEach((client) => {
      if (client !== ws && client.readyState === WebSocket.OPEN) {
        client.send(JSON.stringify(data));
      }
    });
  });

  ws.on('close', () => {
    console.log('Client disconnected');
  });
});

// Запуск HTTP сервера
app.listen(PORT, () => {
  console.log(`HTTP server running on port ${PORT}`);
});

console.log(`WebSocket server running on ws://localhost:8081`);