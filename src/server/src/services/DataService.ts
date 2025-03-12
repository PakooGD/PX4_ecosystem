import { DroneData } from '../models';

class DataService {
  static async save(data: any) {
    try {
     // await DroneData.create(data);
      console.log(`Data saved: ${JSON.stringify(data, null, 2)}`);
    } catch (error) {
      console.error('Failed to save data:', error);
    }
  }
}

export { DataService };


