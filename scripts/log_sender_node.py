import os
import glob
import requests
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

class UlogHandler(Node):
    def __init__(self):
        super().__init__('ulog_handler')
        self.base_log_dir = "/home/alexei/PX4_ecosystem/src/client/PX4-Autopilot/build/px4_sitl_default/rootfs/log" 
        self.server_url = "http://localhost:5000/ulog"  
        self.last_sent_file = None

        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

    def get_latest_log_dir(self, base_dir):
        """Найти последнюю папку с логами в директории."""
        list_of_dirs = glob.glob(os.path.join(base_dir, "*/"))

        if not list_of_dirs:
            return None
        date_dirs = [d for d in list_of_dirs if os.path.basename(os.path.normpath(d)).count('-') == 2]

        if not date_dirs:
            return None
        
        latest_dir = max(date_dirs, key=os.path.getmtime)

        return latest_dir
    
    def get_latest_ulog_file(self, log_dir):
        """Получить последний созданный ulog-файл."""

        list_of_files = glob.glob(os.path.join(log_dir, "*.ulg"))

        if not list_of_files:
            return None
        
        latest_file = max(list_of_files, key=os.path.getctime)

        return latest_file
    
    def send_ulog_file(self, file_path):
        """Отправить ulog-файл на сервер."""
        if not os.path.exists(file_path):
            return False
        try:
            with open(file_path, 'rb') as f:
                files = {'file': f}
                response = requests.post(self.server_url, files=files)
                if response.status_code == 200:
                    return True
                else:
                    return False
        except Exception as e:
            return False

    def monitor_and_send(self):
        """Мониторинг директории и отправка последнего файла."""
        while True:
            latest_dir = self.get_latest_log_dir(self.base_log_dir)
            if latest_dir:
                latest_file = self.get_latest_ulog_file(latest_dir)
                if latest_file and latest_file != self.last_sent_file:
                    if self.send_ulog_file(latest_file):
                        self.last_sent_file = latest_file
                else:
                    self.get_logger().info("No new .ulg files found.")
            else:
                self.get_logger().info("No log directories found.")
            time.sleep(10)  

def main(args=None):
    
    rclpy.init(args=args)
    ulog_handler = UlogHandler()

    monitor_thread = threading.Thread(target=ulog_handler.monitor_and_send, daemon=True)
    monitor_thread.start()
    ulog_handler.get_logger().info("Monitor thread started.")

    executor = MultiThreadedExecutor()
    rclpy.spin(ulog_handler, executor=executor)

    ulog_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()