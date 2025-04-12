import express from 'express';
import { EventEmitter } from 'events';
import { EventTypes } from '../types/ITypes' 
import { AuthController, DroneController } from '../controllers';
import { CryptoService, LogService, AuthService, DroneHandler } from '../services';

export const eventEmitter = new EventEmitter();

eventEmitter.on(EventTypes.RECEIVED_DATA, DroneHandler.HandleData);
eventEmitter.on(EventTypes.LOGOUT, AuthService.HandleLogout);
eventEmitter.on(EventTypes.SET_OFFLINE_STATUS, AuthService.SetAllDronesOffline);
eventEmitter.on(EventTypes.SIGNIN, AuthService.SetOnlineStatus);
eventEmitter.on(EventTypes.UPDATE_DATA, AuthService.UpdateData);
eventEmitter.on(EventTypes.STREAM_DATA, LogService.MavLogStream);
eventEmitter.on(EventTypes.RECIEVE_LOG, LogService.LogReciever);

const router = express.Router();

router.post('/auth', AuthController.AuthDrone);
router.post('/refresh', AuthController.RefreshToken);
router.post('/topics/update', DroneController.HandleTopics)
router.post('/topics/redirect', DroneController.RedirectLogs)
router.get('/drones', AuthController.FetchDrones)
router.get('/log/load', DroneController.loadLogs)

export default router;