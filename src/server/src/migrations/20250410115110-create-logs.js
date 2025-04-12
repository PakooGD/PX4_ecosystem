'use strict';

module.exports = {
  async up(queryInterface, Sequelize) {
    await queryInterface.createTable('logs', {
      id: {
        type: Sequelize.INTEGER,
        primaryKey: true,
        autoIncrement: true
      },
      filename: {
        type: Sequelize.STRING,
        allowNull: false
      },
      date: {
        type: Sequelize.DATE,
        allowNull: false,
        defaultValue: Sequelize.literal('CURRENT_TIMESTAMP')
      },
      compression: {
        type: Sequelize.ENUM('none', 'gzip'),
        allowNull: false,
        defaultValue: 'gzip'
      },
      log_data: {
        type: Sequelize.BLOB,
        allowNull: false
      },
      drone_id: {
        type: Sequelize.UUID,
        allowNull: false,
        references: {
          model: 'drones',
          key: 'drone_id'
        },
        onUpdate: 'CASCADE',
        onDelete: 'CASCADE'
      },
      created_at: {
        type: Sequelize.DATE,
        allowNull: false,
        defaultValue: Sequelize.literal('CURRENT_TIMESTAMP')
      },
      updated_at: {
        type: Sequelize.DATE,
        allowNull: false,
        defaultValue: Sequelize.literal('CURRENT_TIMESTAMP')
      }
    });

    await queryInterface.addIndex('logs', ['drone_id']);
    await queryInterface.addIndex('logs', ['filename']);
  },

  async down(queryInterface) {
    await queryInterface.dropTable('logs');
  }
};
