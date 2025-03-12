import { Sequelize, DataTypes, Model } from 'sequelize';

const sequelize = new Sequelize('postgres://user:password@localhost:5432/database');

class DroneData extends Model {}
DroneData.init({
  topic: { type: DataTypes.STRING, allowNull: false },
  timestamp: { type: DataTypes.BIGINT, allowNull: false },
  data: { type: DataTypes.JSON, allowNull: false },
}, { sequelize, modelName: 'drone_data' });

sequelize.sync();

export { DroneData };