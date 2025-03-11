const express = require('express');
const WebSocket = require('ws');

const app = express();
const port = 3000;

// REST API
app.get('/status', (req, res) => {
  res.json({ status: 'OK' });
});

// WebSocket сервер
const wss = new WebSocket.Server({ port: 8080 });
wss.on('connection', (ws) => {
  ws.on('message', (message) => {
    console.log('Received:', message);
    ws.send('Message received');
  });
});

app.listen(port, () => {
  console.log(`Server running at http://localhost:${port}`);
});