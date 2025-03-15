import { Sequelize } from 'sequelize';
import dotenv from 'dotenv';

dotenv.config();

export const PORTS = {
    INCOMING: 8081,    // Для подключения ROS 2 узла
    OUTGOING: 8082     // Для подключения Foxglove Studio
};

export const TOPIC_WHITELIST = [
    '/vehicle_odometry',
    '/sensor_combined',
    '/vehicle_status'
];

//Database PostgreSQL
export const sequelize = new Sequelize({
  database: process.env.DB_NAME,
  username: process.env.DB_USER,
  password: process.env.DB_PASSWORD,
  host: process.env.DB_HOST,
  port: Number(process.env.DB_PORT),
  dialect: 'postgres',
  logging: false,
});
