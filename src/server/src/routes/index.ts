import { Router } from 'express';
import { DroneController } from '../controllers/DroneController';
import { LogController } from '../controllers/LogController';
import { DroneService } from '../services/DroneService';
import { LogService } from '../services/LogService';
import { WebSocketManager } from '../utils/foxgloveBridge';

const router = Router();
const wsManager = new WebSocketManager(8081);
const droneService = new DroneService(wsManager);
const logService = new LogService();

const droneController = new DroneController(droneService);
const logController = new LogController(logService);

router.post('/data', (req, res) => droneController.handleData(req, res));
router.get('/logs', (req, res) => logController.getLogs(req, res));

export { router };