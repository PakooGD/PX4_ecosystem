import { Sequelize, DataTypes, Model } from 'sequelize';

const sequelize = new Sequelize('database', 'user', 'password', {
  host: 'localhost',
  dialect: 'postgres',
});

class DroneData extends Model {}
DroneData.init({
  topic: { type: DataTypes.STRING, allowNull: false },
  timestamp: { type: DataTypes.BIGINT, allowNull: false },
  data: { type: DataTypes.JSON, allowNull: false },
}, { sequelize, modelName: 'drone_data' });

export { DroneData };