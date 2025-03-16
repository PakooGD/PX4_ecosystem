import { DroneLog } from '../models';
import { WebSocketManager } from '../utils/foxgloveBridge';

class DroneService {
  constructor(private wsManager: WebSocketManager) {}

  async processData(data: any) {
    try {
      // Сохраняем данные в базу
      await DroneLog.create(data);

      // Передаем данные в Foxglove Studio
      this.wsManager.broadcast(data);
    } catch (error) {
      console.error('Error processing data:', error);
    }
  }
}

export { DroneService };