import { DroneData } from '../models';

class DataService {
  static async save(data: any) {
    try {
      //await DroneData.create(data);
      console.log('Data saved to database');
    } catch (error) {
      console.error('Failed to save data:', error);
    }
  }
}

export { DataService };