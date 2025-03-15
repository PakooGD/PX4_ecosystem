import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import math
import importlib
import sys
from collections import defaultdict
import time
from queue import Queue

# Blocklist типов сообщений
UNSUPPORTED_MSG_TYPES = [
]

def camel_to_snake_case(name: str) -> str:
    return ''.join([' ' + char if char.isupper() else char for char in name]).strip().replace(' ', '_').lower()


class PX4WebSocketBridge(Node):
    def __init__(self):
        super().__init__('px4_websocket_bridge')

        self.websocket_url = 'ws://localhost:8082'

        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscribe_to_all_topics()

        self.last_message_time = defaultdict(float)  # Время последнего сообщения для каждого топика
        self.timeout = 5.0  # Таймаут в секундах
        self.ignored_topics = set()  # Список игнорируемых топиков
        self.data_queue = Queue()

    def subscribe_to_all_topics(self):
        """Динамически подписывается на все топики в пакете px4_msgs."""
        package = importlib.import_module('px4_msgs.msg')

        # Получаем список всех типов сообщений в пакете
        msg_types = [msg for msg in dir(package) if msg[0].isupper()]  # Фильтр по заглавной букве
            
        for msg_type_name in msg_types:

            if msg_type_name in UNSUPPORTED_MSG_TYPES:
                continue

            topic_name = self.get_correct_topic_name(f'/fmu/out/{camel_to_snake_case(msg_type_name)}')
            # self.get_logger().info(f"Message topic_name {topic_name}")

            try:
                if not self.is_topic_active(topic_name):
                    continue

                msg_type = getattr(package, msg_type_name)
                
                self.create_subscription(
                    msg_type,
                    topic_name,
                    lambda msg, tn=topic_name: self.generic_callback(msg, tn), 
                    self.qos_profile
                )
            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to {msg_type_name}: {e}")

    def generic_callback(self, msg, topic_name):
        """Универсальный обработчик для всех топиков."""
        name = msg.__class__.__name__

        try:
            # Обновляем время последнего сообщения
            self.last_message_time[topic_name] = time.time()

            data = {
                'name': name,
                'topic': topic_name,
                'timestamp': msg.timestamp if hasattr(msg, 'timestamp') else 0,
                'data': self.extract_data(msg)
            }

            # Отправка данных через WebSocket
            asyncio.get_event_loop().run_until_complete(self.send_data(data))

        except Exception as e:
            self.get_logger().error(f"Error processing {topic_name}: {e}")

    async def send_data_from_queue(self):
        """Асинхронная задача для отправки данных из очереди."""
        while rclpy.ok():
            if not self.data_queue.empty():
                data = self.data_queue.get()
                try:
                    async with websockets.connect(self.websocket_url) as websocket:
                        await websocket.send(json.dumps(data))
                        self.get_logger().info(f"Data sent to WebSocket: {data}")
                except Exception as e:
                    self.get_logger().error(f"WebSocket send error: {e}")
            await asyncio.sleep(1)  # Проверяем очередь каждую секунду

    def monitor_inactive_topics(self):
        """Мониторинг неактивных топиков и добавление их данных в очередь."""
        while rclpy.ok():
            self.check_inactive_topics()
            time.sleep(1)  # Проверяем каждую секунду

    def check_inactive_topics(self):
        """Проверяет неактивные топики и добавляет их данные в очередь."""
        current_time = time.time()
        for topic_name, last_time in self.last_message_time.items():
            if current_time - last_time > self.timeout and topic_name not in self.ignored_topics:
                self.get_logger().warning(f"Topic {topic_name} is inactive (no data for {self.timeout} seconds).")
                self.ignored_topics.add(topic_name)
                # Добавляем данные неактивного топика в очередь
                self.data_queue.put({
                    'name': 'InactiveTopic',
                    'topic': topic_name,
                    'timestamp': int(time.time() * 1000),  # Текущее время в миллисекундах
                    'data': {'status': 'inactive'}
                }) 

    def get_correct_topic_name(self, topic_name):
        """
        Проверяет и возвращает корректное имя топика.
        Если топик не найден с /fmu/out/, проверяет, существует ли он с /fmu/in/.
        """
        active_topics = self.get_topic_names_and_types()

        # Проверяем, существует ли топик с именем, начинающимся на /fmu/out/
        for name, _ in active_topics:
            if name == topic_name:
                return topic_name 

        # Если топик не найден, проверяем, существует ли он с именем, начинающимся на /fmu/in/
        if topic_name.startswith('/fmu/out/'):
            fixed_topic_name = topic_name.replace('/fmu/out/', '/fmu/in/')
            for name, _ in active_topics:
                if name == fixed_topic_name:
                    return fixed_topic_name 
        # self.get_logger().warning(f"Topic {topic_name} not found with either /fmu/out/ or /fmu/in/ prefix.")
        return None

    def is_topic_active(self, topic_name):
        """Проверяет, активен ли топик."""
        try:
            active_topics = self.get_topic_names_and_types()
            for name, _ in active_topics:
                if name == topic_name:
                    return True
            # self.get_logger().warning(f"Topic {topic_name} not found in active topics.")
            return False
        except Exception as e:
            # self.get_logger().error(f"Failed to check topic availability: {e}")
            return False

    async def send_data(self, data):
        """Отправка данных через WebSocket (асинхронная)."""
        try:
            async with websockets.connect(self.websocket_url) as websocket:
                await websocket.send(json.dumps(data))
                self.get_logger().info(f"Data sent to WebSocket: {data}")
        except Exception as e:
            self.get_logger().error(f"WebSocket send error: {e}")

    def extract_data(self, msg):
            """Универсальный метод для извлечения данных из сообщения PX4."""
            data = {}

            if not hasattr(msg, '__slots__'):
                return data
            
            internal_fields = {'_header', '_stamp'}  # Добавьте сюда другие служебные поля, если необходимо

            for field in msg.__slots__:
                if field in internal_fields:
                    continue

                # Убираем _ и капитализируем каждое слово
                field_name = ''.join(word.capitalize() for word in field.lstrip('_').split('_'))
                
                try:
                    value = getattr(msg, field)
                except AttributeError as e:
                    continue

                if (isinstance(value, (float, np.floating)) and math.isnan(value)):
                    value = None

                # Заменяем Infinity, -Infinity и NaN на строки или null
                if isinstance(value, (float, np.floating)):
                    if math.isinf(value):
                        value = "Infinity" if value > 0 else "-Infinity"
                    elif math.isnan(value):
                        value = 'NaN'

                if isinstance(value, (list, tuple, np.ndarray)):
                    if len(value) <= 4:
                        # Для массивов длиной 1–4 используем x, y, z, w
                        field_names = ['x', 'y', 'z', 'w'][:len(value)]
                        data[field_name] = {field_names[i]: float(value[i]) for i in range(len(value))}
                    else:
                        # Для массивов длиной больше 4 используем param1, param2, ...
                        data[field_name] = {f'param{i + 1}': float(value[i]) for i in range(len(value))}
                
                # Обрабатываем вложенные сообщения
                elif hasattr(value, '__slots__'):  
                    data[field_name] = self.extract_data(value)  
                # Простые типы (числа, строки и т.д.)
                else:
                    data[field_name] = value  

            # self.get_logger().info(f"Extracted data: {data}")
            return data        

def main(args=None):
    rclpy.init(args=args)
    px4_websocket_bridge = PX4WebSocketBridge()

    # Запускаем мониторинг неактивных топиков в отдельном потоке
    import threading
    monitor_thread = threading.Thread(target=px4_websocket_bridge.monitor_inactive_topics, daemon=True)
    monitor_thread.start()

    # Запускаем асинхронную задачу для отправки данных из очереди
    asyncio.get_event_loop().create_task(px4_websocket_bridge.send_data_from_queue())

    rclpy.spin(px4_websocket_bridge)
    px4_websocket_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


