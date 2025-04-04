import websockets
import asyncio
import json
from typing import Optional, Callable, Dict, Any, List
from services.crypto_service import CryptoHandler

class WebSocketService:
    def __init__(self, url: str):
        self.crypto = CryptoHandler()
        self.url = url
        self.websocket = None
        self.message_handlers: Dict[str, Callable] = {}
        self._should_reconnect = True
        self._session_initialized = False
        self._listener_task: Optional[asyncio.Task] = None
        self._sender_task: Optional[asyncio.Task] = None
        self.ack_queue = asyncio.Queue()
        self.send_queue = asyncio.Queue()
        self._pending_messages: List[Dict[str, Any]] = []

    async def connect(self, headers: Optional[dict] = None):
        """Подключение к WebSocket серверу"""
        try:
            self.websocket = await websockets.connect(
                self.url,
                additional_headers=headers,
                ping_interval=20,
                ping_timeout=20
            )
            
            # Инициализация сессии
            if not await self._initialize_session():
                raise ConnectionError("Session initialization failed")
                
            # Запускаем задачи для прослушивания и отправки сообщений
            self._listener_task = asyncio.create_task(self._message_listener())
            self._sender_task = asyncio.create_task(self._message_sender())

            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    async def _initialize_session(self):
        """Инициализация безопасной сессии"""
        try:
            # Получаем публичный ключ сервера
            message = await self.websocket.recv()
            data = json.loads(message)
            
            if data.get('type') == 'keyExchange':
                self.crypto.set_server_key(data.get('publicKey'))
                
                # Отправляем инициализацию сессии
                session_data = {
                    'type': 'session_init',
                    **self.crypto.generate_encrypted_session_keys()
                }
                await self.websocket.send(json.dumps(session_data))
                
                # Ждем подтверждения инициализации
                ack = await self.websocket.recv()
                ack_data = json.loads(ack)
                if ack_data.get('type') == 'fetchInfo':
                    self._session_initialized = True
                    
                    # Отправляем все ожидающие сообщения
                    while self._pending_messages:
                        await self.send_queue.put(self._pending_messages.pop(0))
                    
                    return True
        except Exception as e:
            print(f"Session init error: {e}")
        return False

    async def send(self, data: dict) -> bool:
        """Постановка сообщения в очередь на отправку"""
        if not self._session_initialized:
            self._pending_messages.append(data)
            return False
            
        await self.send_queue.put(data)
        return True

    async def _message_sender(self):
        """Отправка сообщений из очереди"""
        while self._should_reconnect:
            try:
                if not self.websocket or not self._session_initialized:
                    await asyncio.sleep(0.1)
                    continue
                    
                data = await self.send_queue.get()
                
                try:
                    encrypted = self.crypto.encrypt_payload(data)
                    await self.websocket.send(encrypted)
                    
                    # Ждем подтверждения с таймаутом
                    try:
                        ack = await asyncio.wait_for(self.ack_queue.get(), timeout=5)
                        ack_data = json.loads(ack)
                        if ack_data.get('type') != 'ack':
                            print("Received non-ACK response")
                            await self.send_queue.put(data)  # Повторная попытка
                    except asyncio.TimeoutError:
                        print("Timeout waiting for ACK")
                        await self.send_queue.put(data)  # Повторная попытка
                        
                except Exception as e:
                    print(f"Send error: {e}")
                    await self.send_queue.put(data)  # Повторная попытка
                    
            except Exception as e:
                print(f"Message sender error: {e}")
                await asyncio.sleep(1)

    def register_message_handler(self, message_type: str, handler: Callable):
        """Регистрация обработчика для определенного типа сообщений"""
        self.message_handlers[message_type] = handler

    async def _message_listener(self):
        """Прослушивание входящих сообщений"""
        while self._should_reconnect and self.websocket:
            try:
                message = await self.websocket.recv()
                data = json.loads(message)
                message_type = data.get('type')

                if message_type == 'ack':
                    await self.ack_queue.put(message)
                elif message_type in self.message_handlers:
                    # Запускаем обработчик в отдельной задаче
                    asyncio.create_task(self._handle_message(message_type, data))
                else:
                    print(f"No handler for message type: {message_type}")
                    
            except websockets.exceptions.ConnectionClosed:
                print("WebSocket connection closed")
                await self._reconnect()
            except json.JSONDecodeError:
                print(f"Failed to decode message: {message}")
            except Exception as e:
                print(f"Error processing message: {e}")

    async def _handle_message(self, message_type: str, data: dict):
        """Обработка сообщения в отдельной задаче"""
        try:
            await self.message_handlers[message_type](data)
        except Exception as e:
            print(f"Error in message handler for {message_type}: {e}")

    async def _reconnect(self):
        """Повторное подключение"""
        if not self._should_reconnect:
            return
            
        print("Attempting to reconnect...")
        await self.disconnect()
        
        # Сохраняем ожидающие сообщения
        pending = []
        while not self.send_queue.empty():
            pending.append(await self.send_queue.get())
        self._pending_messages.extend(pending)
        
        # Пытаемся переподключиться
        while self._should_reconnect:
            try:
                if await self.connect():
                    print("Reconnected successfully")
                    return
                await asyncio.sleep(5)
            except Exception as e:
                print(f"Reconnection failed: {e}")
                await asyncio.sleep(5)

    async def disconnect(self):
        """Закрытие соединения"""
        self._should_reconnect = False
        
        # Отменяем задачи
        if self._listener_task:
            self._listener_task.cancel()
            try:
                await self._listener_task
            except (asyncio.CancelledError, Exception):
                pass
            self._listener_task = None
            
        if self._sender_task:
            self._sender_task.cancel()
            try:
                await self._sender_task
            except (asyncio.CancelledError, Exception):
                pass
            self._sender_task = None
            
        # Закрываем соединение
        if self.websocket:
            try:
                await self.websocket.close()
            except Exception:
                pass
            self.websocket = None
            
        self._session_initialized = False