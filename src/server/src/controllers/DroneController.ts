import { Request, Response } from 'express';
import { DroneService } from '../services';

class DroneController {
  constructor(private droneService: DroneService) {}

  async handleData(req: Request, res: Response) {
    try {
      const data = req.body;
      await this.droneService.processData(data);
      res.status(200).json({ message: 'Data processed' });
    } catch (error) {
      res.status(500).json({ error: 'Failed to process data' });
    }
  }
}

export { DroneController };