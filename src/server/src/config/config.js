require('dotenv').config();

module.exports = {
  development: {
    database: process.env.DB_NAME || 'drone_db',
    username: process.env.DB_USER || 'postgres',
    password: process.env.DB_PASSWORD || 'dsk_supply_password',
    host: process.env.DB_HOST || 'localhost',
    port: process.env.DB_PORT || 5432,
    dialect: 'postgres',
    migrationStorageTableName: 'sequelize_meta',
    seederStorageTableName: 'sequelize_seeders'
  },
  test: {
    // тестовая конфигурация
  },
  production: {
    // production конфигурация
  }
};