import websockets
import asyncio
import json
from typing import Optional, Callable, Dict, Any
from services.crypto_service import CryptoHandler

class WebSocketService:
    def __init__(self, url: str):
        self.crypto = CryptoHandler()
        self.url = url
        self.websocket = None
        self.message_handlers: Dict[str, Callable] = {}
        self._should_reconnect = True
        self._session_initialized = False

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
                    return True
        except Exception as e:
            print(f"Session init error: {e}")
        return False

    async def send(self, data: dict) -> bool:
        """Отправка зашифрованного сообщения"""
        if not self._session_initialized or not self.websocket:
            return False
            
        try:
            encrypted = self.crypto.encrypt_payload(data)
            await self.websocket.send(encrypted)
            
            # Ждем подтверждения
            ack = await asyncio.wait_for(self.websocket.recv(), timeout=5)
            ack_data = json.loads(ack)
            return ack_data.get('type') == 'ack'
        except asyncio.TimeoutError:
            print("Timeout waiting for ACK")
            return False
        except Exception as e:
            print(f"Send error: {e}")
            return False

    async def disconnect(self):
        """Закрытие соединения"""
        self._should_reconnect = False
        if self.websocket:
            await self.websocket.close()
            self.websocket = None
        self._session_initialized = False

    def register_message_handler(self, message_type: str, handler: Callable):
        """Регистрация обработчика сообщений"""
        self.message_handlers[message_type] = handler