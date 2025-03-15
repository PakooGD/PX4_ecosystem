import { DroneLog } from '../models';

const { Op } = require('sequelize')

class LogService {
  async getLogsByDate(date: string) {
    const startOfDay = new Date(date).setHours(0, 0, 0, 0);
    const endOfDay = new Date(date).setHours(23, 59, 59, 999);

    return DroneLog.findAll({
      where: {
        timestamp: {
          [Op.between]: [startOfDay, endOfDay],
        },
      },
    });
  }
}

export { LogService };