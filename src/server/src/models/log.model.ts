import { Table, Column, Model, ForeignKey, BelongsTo, DataType } from 'sequelize-typescript';
import { Drone } from './';
import { CompressionType } from '../types/ITypes';

@Table({ tableName: 'logs' })
export class Log extends Model {
  @Column({
    type: DataType.STRING,
    allowNull: false
  })
  filename!: string;

  @Column({
    type: DataType.DATE,
    allowNull: false,
    defaultValue: DataType.NOW
  })
  date!: Date;

  @Column({
    type: DataType.ENUM('none', 'gzip'),
    allowNull: false,
    defaultValue: 'gzip'
  })
  compression!: CompressionType;

  @Column({
    type: DataType.BLOB('long'),
    allowNull: false
  })
  log_data!: Buffer;

  @ForeignKey(() => Drone)
  @Column({
    type: DataType.UUID,
    allowNull: false
  })
  drone_id!: string;

  @BelongsTo(() => Drone)
  drone!: Drone;
}