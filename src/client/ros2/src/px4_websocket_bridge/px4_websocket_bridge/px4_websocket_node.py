import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    VehicleOdometry,  # Положение и скорость
    SensorCombined,  # Данные с датчиков (IMU)
    VehicleStatus,   # Статус дрона
    VehicleCommand,  # Команды управления
    BatteryStatus,   # Состояние батареи
    VehicleAttitude, # Ориентация дрона
)
import asyncio
import websockets
import json

class PX4WebSocketBridge(Node):
    def __init__(self):
        super().__init__('px4_websocket_bridge')

        # Адрес WebSocket сервера
        self.websocket_url = 'ws://localhost:8081'

        # Подписка на топики PX4
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            10)

        self.sensor_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_callback,
            10)

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            10)

        self.command_sub = self.create_subscription(
            VehicleCommand,
            '/fmu/out/vehicle_command',
            self.command_callback,
            10)

        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_callback,
            10)

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            10)

    async def send_data(self, data):
        """Отправка данных через WebSocket."""
        async with websockets.connect(self.websocket_url) as websocket:
            await websocket.send(json.dumps(data))
            self.get_logger().info(f"Data sent: {data}")

    def odometry_callback(self, msg):
        """Обработка данных о положении и скорости."""
        data = {
            'topic': '/vehicle_odometry',
            'timestamp': msg.timestamp,
            'position': {
                'x': msg.position[0],
                'y': msg.position[1],
                'z': msg.position[2]
            },
            'velocity': {
                'vx': msg.velocity[0],
                'vy': msg.velocity[1],
                'vz': msg.velocity[2]
            }
        }
        asyncio.get_event_loop().run_until_complete(self.send_data(data))

    def sensor_callback(self, msg):
        """Обработка данных с датчиков (IMU)."""
        data = {
            'topic': '/sensor_combined',
            'timestamp': msg.timestamp,
            'gyro': {
                'x': msg.gyro_rad[0],
                'y': msg.gyro_rad[1],
                'z': msg.gyro_rad[2]
            },
            'accelerometer': {
                'x': msg.accelerometer_m_s2[0],
                'y': msg.accelerometer_m_s2[1],
                'z': msg.accelerometer_m_s2[2]
            }
        }
        asyncio.get_event_loop().run_until_complete(self.send_data(data))

    def status_callback(self, msg):
        """Обработка статуса дрона."""
        data = {
            'topic': '/vehicle_status',
            'timestamp': msg.timestamp,
            'arming_state': msg.arming_state,
            'nav_state': msg.nav_state,
            'system_status': msg.system_status
        }
        asyncio.get_event_loop().run_until_complete(self.send_data(data))

    def command_callback(self, msg):
        """Обработка команд управления."""
        data = {
            'topic': '/vehicle_command',
            'timestamp': msg.timestamp,
            'command': msg.command,
            'param1': msg.param1,
            'param2': msg.param2,
            'param3': msg.param3,
            'param4': msg.param4
        }
        asyncio.get_event_loop().run_until_complete(self.send_data(data))

    def battery_callback(self, msg):
        """Обработка данных о батарее."""
        data = {
            'topic': '/battery_status',
            'timestamp': msg.timestamp,
            'voltage': msg.voltage_v,
            'current': msg.current_a,
            'remaining': msg.remaining
        }
        asyncio.get_event_loop().run_until_complete(self.send_data(data))

    def attitude_callback(self, msg):
        """Обработка данных об ориентации."""
        data = {
            'topic': '/vehicle_attitude',
            'timestamp': msg.timestamp,
            'orientation': {
                'x': msg.q[0],
                'y': msg.q[1],
                'z': msg.q[2],
                'w': msg.q[3]
            }
        }
        asyncio.get_event_loop().run_until_complete(self.send_data(data))

def main(args=None):
    rclpy.init(args=args)
    px4_websocket_bridge = PX4WebSocketBridge()
    rclpy.spin(px4_websocket_bridge)
    px4_websocket_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()