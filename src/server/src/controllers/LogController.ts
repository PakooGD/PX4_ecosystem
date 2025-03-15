import { Request, Response } from 'express';
import { LogService } from '../services';

class LogController {
  constructor(private logService: LogService) {}

  async getLogs(req: Request, res: Response) {
    try {
      const { date } = req.query;
      const logs = await this.logService.getLogsByDate(date as string);
      res.status(200).json(logs);
    } catch (error) {
      res.status(500).json({ error: 'Failed to fetch logs' });
    }
  }
}

export { LogController };