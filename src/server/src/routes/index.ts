import { Router } from 'express';
import { DataController } from '../controllers';

const router = Router();

router.post('/data', DataController.saveData);

export { router };