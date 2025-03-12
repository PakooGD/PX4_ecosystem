import { Request, Response } from 'express';
import { DataService } from '../services';

class DataController {
  static async saveData(req: Request, res: Response) {
    try {
      const data = req.body;
      DataService.save(data);
      res.status(200).json({ message: 'Data saved' });
    } catch (error) {
      res.status(500).json({ error: 'Failed to save data' });
    }
  }
}

export { DataController };