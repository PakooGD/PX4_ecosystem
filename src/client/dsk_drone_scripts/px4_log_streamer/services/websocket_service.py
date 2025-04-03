import asyncio
import websockets
import json
from collections import deque
from typing import Optional, Callable, Dict, Any
from services.crypto_service import CryptoHandler

class WebSocketService:
    def __init__(self, url: str, ping_interval: int = 20, ping_timeout: int = 20):
        self.crypto = CryptoHandler()
        self.url = url
        self.ping_interval = ping_interval
        self.ping_timeout = ping_timeout
        
        self.websocket = None
        self.message_queue = deque()
        self.ack_queue = asyncio.Queue()
        self.message_handlers: Dict[str, Callable] = {}
        self.connection_callbacks = []
        self.disconnection_callbacks = []
        
        self._process_task: Optional[asyncio.Task] = None
        self._should_reconnect = True
        self._retry_delay = 1
        self._session_initialized = False


    async def connect(self, headers: Optional[dict] = None, max_retries: int = None, 
                     require_session_init: bool = True):
        """Подключение к WebSocket серверу с инициализацией сессии"""
        self._should_reconnect = True
        retry_count = 0
        
        while self._should_reconnect and (max_retries is None or retry_count < max_retries):
            try:
                async with websockets.connect(
                    self.url,
                    additional_headers=headers,
                    ping_interval=self.ping_interval,
                    ping_timeout=self.ping_timeout,
                ) as websocket:
                    self.websocket = websocket
                    self._retry_delay = 1
                    retry_count = 0
                    
                    if require_session_init:
                        if not await self._initialize_session():
                            raise ConnectionError("Не удалось инициализировать сессию")
                    
                    await self._on_connected()
                    self._process_task = asyncio.create_task(self._process_outgoing_messages())
                    
                    try:
                        async for message in websocket:
                            await self._handle_message(message)
                    except websockets.exceptions.ConnectionClosed as e:
                        await self._on_disconnected(f"Соединение закрыто: {e}")
                    except Exception as e:
                        await self._on_disconnected(f"Неожиданная ошибка: {e}")
                    finally:
                        await self._cleanup()
                        
            except Exception as e:
                await self._on_connection_failed(f"Ошибка подключения: {e}")
                retry_count += 1
                
                if max_retries is not None and retry_count >= max_retries:
                    break
                
                await asyncio.sleep(self._retry_delay)
                self._retry_delay = min(self._retry_delay * 2, 60)

    async def _initialize_session(self):
        """Initialize secure session"""
        try:
            data = await self.websocket.recv()
            if json.loads(data).get('type') == 'keyExchange':
                public_key = json.loads(data).get('publicKey')
                self.crypto.set_server_key(public_key)
                session_init = {
                    'type': 'session_init',
                    **self.crypto.generate_encrypted_session_keys()
                }
                await self.websocket.send(json.dumps(session_init))
                print("Session initialized successfully")
                return True
        except Exception as e:
            print(f"Error initializing session: {e}")
            raise
        return False

    async def disconnect(self):
        """Disconnect from WebSocket server"""
        self._should_reconnect = False
        if self.websocket:
            await self.websocket.close()
        if self._process_task:
            self._process_task.cancel()

    async def send(self, data: dict, require_ack: bool = True, max_retries: int = 3) -> bool:
        """Send data through WebSocket with optional retry logic"""
        try:
            encrypted = self.crypto.encrypt_payload(data)
            self.message_queue.append((encrypted, require_ack, max_retries))
            return True
        except Exception as e:
            print(f"Error queueing message: {e}")
            return False

    def register_message_handler(self, message_type: str, handler: Callable):
        """Register a handler for specific message type"""
        self.message_handlers[message_type] = handler

    def register_connection_callback(self, callback: Callable):
        """Register callback for connection events"""
        self.connection_callbacks.append(callback)

    def register_disconnection_callback(self, callback: Callable):
        """Register callback for disconnection events"""
        self.disconnection_callbacks.append(callback)

    async def _on_connected(self):
        """Handle connection established"""
        for callback in self.connection_callbacks:
            try:
                await callback()
            except Exception as e:
                print(f"Error in connection callback: {e}")

    async def _on_disconnected(self, reason: str):
        """Handle disconnection"""
        for callback in self.disconnection_callbacks:
            try:
                await callback(reason)
            except Exception as e:
                print(f"Error in disconnection callback: {e}")

    async def _on_connection_failed(self, reason: str):
        """Handle connection failure"""
        print(f"Connection failed: {reason}")
        await asyncio.sleep(self._retry_delay)

    async def _cleanup(self):
        """Clean up resources"""
        if self._process_task:
            self._process_task.cancel()
            try:
                await self._process_task
            except asyncio.CancelledError:
                pass
        self.websocket = None

    async def _process_outgoing_messages(self):
        """Process messages from the outgoing queue"""
        while self.websocket and self._should_reconnect:
            try:
                if self.message_queue:
                    data, require_ack, max_retries = self.message_queue.popleft()
                    if require_ack:
                        await self._send_with_retry(data, max_retries)
                    else:
                        await self.websocket.send(data)
                await asyncio.sleep(0.01)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Error processing outgoing messages: {e}")
                await asyncio.sleep(1)

    async def _send_with_retry(self, data: str, max_retries: int) -> bool:
        """Send data with retry logic"""
        for attempt in range(max_retries):
            try:
                if not self.websocket:
                    return False

                await self.websocket.send(data)
                try:
                    ack = await asyncio.wait_for(self.ack_queue.get(), timeout=5)
                    if ack.get('type') == 'ack':
                        return True
                    print(f"Invalid ACK received: {ack}")
                except asyncio.TimeoutError:
                    print(f"ACK timeout on attempt {attempt+1}")
                
            except websockets.exceptions.ConnectionClosed:
                return False
            except Exception as e:
                print(f"Unexpected error in attempt {attempt+1}: {str(e)}")

            delay = min(1 * (2 ** attempt), 30)
            print(f"Retrying in {delay} seconds...")
            await asyncio.sleep(delay)
        
        print(f"All {max_retries} attempts failed")
        return False

    async def _handle_message(self, message: str):
        """Handle incoming messages"""
        try:
            data = json.loads(message)
            message_type = data.get('type')
            
            if message_type == 'ack':
                await self.ack_queue.put(data)
                return
                
            if message_type in self.message_handlers:
                await self.message_handlers[message_type](data)
            else:
                print(f"No handler for message type: {message_type}")
        except json.JSONDecodeError:
            print(f"Failed to decode message: {message}")
        except Exception as e:
            print(f"Error processing message: {e}")