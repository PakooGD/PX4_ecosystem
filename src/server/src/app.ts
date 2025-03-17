import { WebSocketServer } from 'ws';
import express, { Request, Response } from "express";
import { FoxgloveServer } from '@foxglove/ws-protocol';
import { sendData, processUlogFile, processSchemas,getSchemaNameByChannelId } from "./utils";
import WebSocket from 'ws';

const path = require('path');
const fs = require('fs');
const multer = require('multer');

const FoxgloveStudioPort = 8081;
const BridgePort = 8082;
const httpPort = 3000;

const readUlog = false;
let topicSchema;

const app = express();

const storage = multer.diskStorage({
    destination: (req:any, file:any, cb:any) => {
        const uploadDir = path.join(__dirname, 'ulog');
        if (!fs.existsSync(uploadDir)) {
            fs.mkdirSync(uploadDir, { recursive: true }); 
        }
        cb(null, uploadDir); 
    },
    filename: (req:any, file:any, cb:any) => {
        cb(null, file.originalname);
    },
});

let uploadedFile:any = null;

const upload = multer({ storage });

app.use(express.json()); 
const server = new FoxgloveServer({ name: "px4-foxglove-bridge" });

const ws = new WebSocketServer({
    port: FoxgloveStudioPort,
    handleProtocols: (protocols) => server.handleProtocols(protocols),
});

app.listen(httpPort, () => {
    console.log(`Server is running on http://localhost:${httpPort}`);
});

app.post('/mavlog', upload.single('file'), (req: any, res: any) => {
    if (!req.file) {
        return res.status(400).send("No file uploaded.");
    }

    const filePath = req.file.path;
    const originalName = req.file.originalname;

    if(readUlog){
        processUlogFile(server,filePath);
    }
    // Перемещение файла в постоянную директорию
    const targetPath = path.join(__dirname, 'log/mav', originalName);
    fs.rename(filePath, targetPath, (err:any) => {
        if (err) {
            return res.status(500).send("Failed to save the file.");
        }
        res.status(200).send({ message: "File uploaded successfully.", filePath: targetPath });
    });
});

app.post('/schema', (req: any, res: any) => {
    topicSchema = req.body;
    if (!topicSchema) {
        return res.status(400).send("Missing required fields: topic, schemaName, schema");
    }
    const channelId = processSchemas(server, topicSchema)
    res.status(200).send({ channelId });
});

app.post('/ulog', upload.single('file'), (req: any, res: any) => {
    if (!req.file) {
        return res.status(400).send("No file uploaded.");
    }
    uploadedFile = req.file;
    if(readUlog){
        processUlogFile(server,uploadedFile.path);
    }

    res.status(200).send({
        message: "File uploaded successfully.",
        fileInfo: {
            originalName: uploadedFile.originalname,
            path: uploadedFile.path,
            size: uploadedFile.size,
        },
    });
});



function sendToPythonBridge(data: { type: string; topic: string }) {
    if (pythonBridgeWebSocket) {
        pythonBridgeWebSocket.send(JSON.stringify(data));
    } else {
        console.error("Python Bridge WebSocket is not connected.");
    }
}

let pythonBridgeWebSocket: WebSocket | null = null;


ws.on("connection", (conn, req) => {
    console.log(`🎮 Foxglove Studio connected on ws://localhost:${FoxgloveStudioPort}`);
    const name = `${req.socket.remoteAddress}:${req.socket.remotePort}`;
   
    if(!readUlog) {
        const wss = new WebSocketServer({ port: BridgePort }); 
        wss.on('connection', (ws) => {
            pythonBridgeWebSocket = ws;
            ws.on('message', (message: string) => {
                sendData(server, message);
            });
            ws.on('close', () => {
                console.log("Python Bridge WebSocket disconnected");
                pythonBridgeWebSocket = null; 
            });
            ws.on('error', (error) => {
                console.error("WebSocket error:", error);
                pythonBridgeWebSocket = null;  
            });
        });
    }

    server.on("subscribe", (chanId) => {
        const shemaName = getSchemaNameByChannelId(chanId);
        if (shemaName) {
            sendToPythonBridge({ type: 'subscribe', topic:shemaName });
        }
    });

    server.on("unsubscribe", (chanId) => {
        const shemaName = getSchemaNameByChannelId(chanId);
        if (shemaName) {
            sendToPythonBridge({ type: 'unsubscribe', topic:shemaName });
        }
    });

    server.on("error", (err) => {
        console.error("server error: %o", err);
    });

    server.handleConnection(conn, name);
});



