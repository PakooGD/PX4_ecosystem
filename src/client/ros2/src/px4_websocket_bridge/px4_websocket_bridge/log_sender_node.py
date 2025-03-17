import os
import glob
import requests
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy

class UlogHandler(Node):
    def __init__(self):
        super().__init__('ulog_handler')
        self.base_log_dir = "/home/alexei/PX4_ecosystem/src/client/PX4-Autopilot/build/px4_sitl_default/rootfs/log"  # Укажите путь к директории с ulog-файлами
        self.server_url = "http://localhost:3000/ulog"  # Укажите URL сервера
        self.last_sent_file = None

        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Подписка на топик vehicle_status
        self.subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            self.qos_profile
        )

    # mavftp --device=udp:127.0.0.1:14550 --target-system=1 --target-component=1 --download /fs/microsd/log/*.ulg /path/to/local/directory

    def get_latest_log_dir(self, base_dir):
        """Найти последнюю папку с логами в директории."""
        self.get_logger().info(f"Checking directory: {base_dir}")
        list_of_dirs = glob.glob(os.path.join(base_dir, "*/"))
        self.get_logger().info(f"Found directories: {list_of_dirs}")
        if not list_of_dirs:
            return None
        date_dirs = [d for d in list_of_dirs if os.path.basename(os.path.normpath(d)).count('-') == 2]
        self.get_logger().info(f"Date directories: {date_dirs}")
        if not date_dirs:
            return None
        latest_dir = max(date_dirs, key=os.path.getmtime)
        self.get_logger().info(f"Latest directory: {latest_dir}")
        return latest_dir
    
    def get_latest_ulog_file(self, log_dir):
        """Получить последний созданный ulog-файл."""
        self.get_logger().info(f"Checking for .ulg files in: {log_dir}")
        list_of_files = glob.glob(os.path.join(log_dir, "*.ulg"))
        self.get_logger().info(f"Found .ulg files: {list_of_files}")
        if not list_of_files:
            return None
        latest_file = max(list_of_files, key=os.path.getctime)
        self.get_logger().info(f"Latest .ulg file: {latest_file}")
        return latest_file
    
    def send_ulog_file(self, file_path):
        """Отправить ulog-файл на сервер."""
        if not os.path.exists(file_path):
            self.get_logger().error(f"File {file_path} does not exist.")
            return False
        try:
            self.get_logger().info(f"Attempting to send file: {file_path}")
            with open(file_path, 'rb') as f:
                files = {'file': f}
                response = requests.post(self.server_url, files=files)
                self.get_logger().info(f"Server response: {response.status_code}, {response.text}")
                if response.status_code == 200:
                    self.get_logger().info(f"File {file_path} sent successfully.")
                    return True
                else:
                    self.get_logger().error(f"Failed to send file {file_path}: {response.status_code}")
                    return False
        except Exception as e:
            self.get_logger().error(f"Error sending file {file_path}: {e}")
            return False

    def vehicle_status_callback(self, msg):
        """Обработчик изменения статуса арминга."""
        self.get_logger().info(f"Received vehicle_status message: arming_state={msg.arming_state}")
        if not msg.arming_state:  # Если статус арминга false
            self.get_logger().info("Arming state changed to false. Sending latest ulog file.")
            latest_dir = self.get_latest_log_dir(self.base_log_dir)
            if latest_dir:
                latest_file = self.get_latest_ulog_file(latest_dir)
                if latest_file and latest_file != self.last_sent_file:
                    if self.send_ulog_file(latest_file):
                        self.last_sent_file = latest_file

    def monitor_and_send(self):
        """Мониторинг директории и отправка последнего файла."""
        self.get_logger().info("Starting monitor_and_send thread.")
        while True:
            self.get_logger().info("Checking for latest log directory...")
            latest_dir = self.get_latest_log_dir(self.base_log_dir)
            if latest_dir:
                self.get_logger().info(f"Latest log directory found: {latest_dir}")
                latest_file = self.get_latest_ulog_file(latest_dir)
                if latest_file and latest_file != self.last_sent_file:
                    self.get_logger().info(f"New .ulg file found: {latest_file}")
                    if self.send_ulog_file(latest_file):
                        self.last_sent_file = latest_file
                else:
                    self.get_logger().info("No new .ulg files found.")
            else:
                self.get_logger().info("No log directories found.")
            time.sleep(10)  # Проверка каждые 10 секунд

def main(args=None):
    
    rclpy.init(args=args)
    ulog_handler = UlogHandler()

    # Запуск мониторинга директории в отдельном потоке
    monitor_thread = threading.Thread(target=ulog_handler.monitor_and_send, daemon=True)
    monitor_thread.start()
    ulog_handler.get_logger().info("Monitor thread started.")

    # Запуск ROS 2 узла
    executor = MultiThreadedExecutor()
    rclpy.spin(ulog_handler, executor=executor)

    # Очистка
    ulog_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()