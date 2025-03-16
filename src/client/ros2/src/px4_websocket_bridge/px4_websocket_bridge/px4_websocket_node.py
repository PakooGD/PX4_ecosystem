import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import math
import importlib
from collections import defaultdict
import time
import requests

# Blocklist типов сообщений
UNSUPPORTED_MSG_TYPES = []

def camel_to_snake_case(name: str) -> str:
    return ''.join([' ' + char if char.isupper() else char for char in name]).strip().replace(' ', '_').lower()

def topic_to_camel(topic: str) -> str:
    if topic.startswith('/fmu/out/'):
        topic = topic[len('/fmu/out/'):]

    camelName = ''.join(word.capitalize() for word in topic.split('_'))
    
    return camelName

class PX4WebSocketBridge(Node):
    def __init__(self):
        super().__init__('px4_websocket_bridge')

        self.websocket_url = 'ws://localhost:8082'
        self.http_url = 'http://localhost:3000/schema'

        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.schemas_sent = set()  # Множество для хранения отправленных схем

        self.get_all_topics()

    def get_all_topics(self):
        """Динамически подписывается на все топики в пакете px4_msgs."""
        package = importlib.import_module('px4_msgs.msg')

        message_names = [msg for msg in dir(package) if msg[0].isupper()]  # Фильтр по заглавной букве
            
        for name in message_names:
            if name in UNSUPPORTED_MSG_TYPES:
                continue
     
            topic_name = self.get_topic_name(name)
            
            if not self.is_topic_active(topic_name):
                continue

            msg = getattr(package, name)

            schema = self.generate_schema(msg)
            self.send_schema(topic_name, name, schema)

    async def handle_websocket(self):
        """Обработка WebSocket соединения."""
        async with websockets.connect(self.websocket_url) as websocket:
            self.websocket = websocket
            async for message in websocket:
                data = json.loads(message)
                if data.get('type') == 'subscribe':
                    topic_name = data.get('topic')
                    self.subscribe(topic_name)
                elif data.get('type') == 'unsubscribe':
                    topic_name = data.get('topic')
                    self.unsubscribe(topic_name)

    def subscribe(self, topic_name):
        """Подписка на топик."""
        if topic_name in [sub.topic_name for sub in self.subscriptions]:
            self.get_logger().warning(f"Already subscribed to {topic_name}")
            return

        package = importlib.import_module('px4_msgs.msg')
      
        try:
            if not self.is_topic_active(topic_name):
                self.get_logger().error(f"Not an active topic {topic_name}: {e}")
                return

            name = topic_to_camel(topic_name)
            msg_type = getattr(package, name)
        
            # Создаем подписку
            self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, tn=topic_name: self.generic_callback(msg, tn), 
                self.qos_profile
            )
            self.get_logger().info(f"Subscribed as {topic_name}")

        except AttributeError as e:
            self.get_logger().error(f"Failed to find message type {topic_name}: {e}")
            return  

    def unsubscribe(self, topic_name):
        """Отписка от топика."""
        if not topic_name:
            self.get_logger().error(f"Topic name not found for {topic_name}")
            return

        # Ищем подписку по имени топика
        for sub in self.subscriptions:
            if sub.topic_name == topic_name:
                # Уничтожаем подписку
                self.destroy_subscription(sub)
                self.get_logger().info(f"Unsubscribed as {topic_name}")
                return
        self.get_logger().warning(f"No subscription found for {topic_name}")

    def generic_callback(self, msg, topic_name):
        """Универсальный обработчик для всех топиков."""
        self.get_logger().info(f"Callback triggered for {topic_name}")
        try:
            data = {
                'name': msg.__class__.__name__,
                'topic': topic_name,
                'timestamp': msg.timestamp if hasattr(msg, 'timestamp') else 0,
                'data': self.extract_data(msg)
            }
            self.get_logger().info(f"Extracted data: {data}") 
            asyncio.create_task(self.send_data(data))

        except Exception as e:
            self.get_logger().error(f"Error processing {topic_name}: {e}")

    async def send_data(self, data):
        """Отправка данных через WebSocket (асинхронная)."""
        try:
            await self.websocket.send(json.dumps(data))
            # self.get_logger().info(f"Data sent to WebSocket: {data}")
        except Exception as e:
            self.get_logger().error(f"WebSocket send error: {e}")

    def get_topic_name(self, name):
        """
        Проверяет и возвращает корректное имя топика.
        Если топик не найден с /fmu/out/, проверяет, существует ли он с /fmu/in/.
        """
        topic = f'/fmu/out/{camel_to_snake_case(name)}'
        active_topics = self.get_topic_names_and_types()

        for active_topic, _ in active_topics:
            if active_topic == topic:
                return topic 

        if topic.startswith('/fmu/out/'):
            fixed_topic_name = topic.replace('/fmu/out/', '/fmu/in/')
            for active_topic, _ in active_topics:
                if active_topic == fixed_topic_name:
                    return fixed_topic_name 
        return None

    def is_topic_active(self, topic_name):
        """Проверяет, активен ли топик."""
        try:
            active_topics = self.get_topic_names_and_types()
            for name, _ in active_topics:
                if name == topic_name:
                    return True
            return False
        except Exception as e:
            return False

    def generate_schema(self, msg):
        """Генерация схемы для типа сообщения."""
        schema = {"type": "object", "properties": {}}

        if not hasattr(msg, '__slots__'):
            return schema
        
        internal_fields = {'_header', '_stamp'}

        for field in msg.__slots__:
            if field in internal_fields:
                continue

            field_name = ''.join(word.capitalize() for word in field.lstrip('_').split('_'))
            value = getattr(msg, field)

            if isinstance(value, (int, float, np.floating)):
                schema["properties"][field_name] = {"type": "number"}
                if math.isnan(value) or math.isinf(value):
                    schema["properties"][field_name] = {"type": "string"}
                else:
                    schema["properties"][field_name] = {"type": "number"}
            elif isinstance(value, (list, tuple, np.ndarray)):
                if isinstance(value[0], (int, float, np.floating)):
                    schema["properties"][field_name] = {"type": "array", "items": {"type": "number"}}
                elif isinstance(value[0], str):
                    schema["properties"][field_name] = {"type": "array", "items": {"type": "string"}}
                else:
                    schema["properties"][field_name] = {"type": "array", "items": self.generate_schema(value[0])}
            elif hasattr(value, '__slots__'):  
                schema["properties"][field_name] = self.generate_schema(value)
            else:
                schema["properties"][field_name] = {"type": "string"} 

        return schema

    def send_schema(self, name, topic_name, schema):
        """Отправка схемы на сервер через HTTP-запрос с использованием requests."""
        if name in self.schemas_sent:
            return 
        try:
            response = requests.post(self.http_url, json={"topic": topic_name, "encoding": "json", "schemaName": name, "schema": schema})
            if response.status_code == 200:
                self.schemas_sent.add(name)
                self.get_logger().info(f"Schema for {name} sent successfully.")
            else:
                self.get_logger().error(f"Failed to send schema for {name}: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error sending schema for {name}: {e}")

    def extract_data(self, msg):
        """Универсальный метод для извлечения данных из сообщения PX4."""
        data = {}

        if not hasattr(msg, '__slots__'):
            return data
        
        internal_fields = {'_header', '_stamp'}  # Добавьте сюда другие служебные поля, если необходимо

        for field in msg.__slots__:
            if field in internal_fields:
                continue

            field_name = ''.join(word.capitalize() for word in field.lstrip('_').split('_'))
            value = getattr(msg, field)

            if isinstance(value, (int, float, np.floating)):
                if math.isnan(value):
                    data[field_name] = 'Nan'
                elif math.isinf(value):
                    data[field_name] = "Infinity" if value > 0 else "-Infinity"
                else:
                    data[field_name] = value
            elif isinstance(value, (list, tuple, np.ndarray)):
                if isinstance(value[0], (int, float, np.floating)):
                    data[field_name] = [float(v) for v in value]
                elif isinstance(value[0], str):
                    data[field_name] = list(value)
                else:
                    data[field_name] = [self.extract_data(v) for v in value] 
            elif hasattr(value, '__slots__'):  
                data[field_name] = self.extract_data(value)       
            else:
                data[field_name] = str(value)

        return data        

async def spin_node(node):
    """Асинхронный spin для ROS 2 узла."""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    px4_websocket_bridge = PX4WebSocketBridge()

    # Запуск асинхронного цикла событий
    loop = asyncio.get_event_loop()
    loop.create_task(spin_node(px4_websocket_bridge))
    loop.run_until_complete(px4_websocket_bridge.handle_websocket())

    # Остановка цикла событий
    loop.close()
    px4_websocket_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

