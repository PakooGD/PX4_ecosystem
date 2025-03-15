import { DataTypes, Model, Sequelize } from 'sequelize';

class DroneLog extends Model {
  public id!: number;
  public topic!: string;
  public timestamp!: number;
  public data!: object;
}

const initDroneLogModel = (sequelize: Sequelize) => {
  DroneLog.init(
    {
      id: {
        type: DataTypes.INTEGER,
        primaryKey: true,
        autoIncrement: true,
      },
      topic: {
        type: DataTypes.STRING,
        allowNull: false,
      },
      timestamp: {
        type: DataTypes.BIGINT,
        allowNull: false,
      },
      data: {
        type: DataTypes.JSON,
        allowNull: false,
      },
    },
    {
      sequelize,
      modelName: 'DroneLog',
      tableName: 'drone_logs',
    }
  );
};

export { DroneLog, initDroneLogModel };