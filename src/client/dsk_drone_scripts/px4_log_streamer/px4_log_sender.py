import os
import glob
import time
import threading
import json
import base64
import zlib
import asyncio
import websockets
from services.crypto_service import CryptoHandler
from services.auth_service import AuthHandler
from typing import Optional, Tuple
import fcntl

class UlogHandler:
    def __init__(self):
        self.base_log_dirs = [
            "/home/alexei/Projects/PX4_ecosystem/src/client/PX4-Autopilot/build/px4_sitl_default/rootfs/log",  # Локальная тестовая директория
            "/fs/microsd/log"  # SD-карта на дроне
        ]
        self.server_url = 'ws://localhost:8082'  # WebSocket endpoint
        self.last_sent_file = None
        self.crypto = CryptoHandler()
        self.auth = AuthHandler()
        self.chunk_size = 1024 * 1024  # 1MB chunks
        self.websocket = None
        self._session_initialized = False
        self._pending_messages = []

    async def _initialize_session(self):
        """Инициализация безопасной сессии"""
        try:
            message = await self.websocket.recv()
            data = json.loads(message)
            
            if data.get('type') == 'keyExchange':
                self.crypto.set_server_key(data.get('publicKey'))
                session_data = {
                    'type': 'session_init',
                    **self.crypto.generate_encrypted_session_keys()
                }
                await self.websocket.send(json.dumps(session_data))
                
                ack = await self.websocket.recv()
                ack_data = json.loads(ack)
                if ack_data.get('type') == 'session_ack':
                    print("session initialized")
                    self._session_initialized = True
                    
                    return True
        except Exception as e:
            print(f"Session init error: {e}")
        return False

    def find_log_dirs(self) -> list:
        """Находит все доступные директории с логами"""
        available_dirs = []
        for dir_path in self.base_log_dirs:
            if os.path.exists(dir_path):
                available_dirs.append(dir_path)
        return available_dirs

    def get_latest_log_dir(self) -> Optional[str]:
        """Находит последнюю папку с логами"""
        for base_dir in self.find_log_dirs():
            try:
                dirs = glob.glob(os.path.join(base_dir, "*/"))
                date_dirs = [d for d in dirs if os.path.basename(os.path.normpath(d)).count('-') == 2]
                if date_dirs:
                    return max(date_dirs, key=os.path.getmtime)
            except Exception as e:
                print(f"Error checking {base_dir}: {e}")
        return None

    def get_latest_ulog_file(self) -> Optional[str]:
        """Находит последний ulog-файл"""
        log_dir = self.get_latest_log_dir()
        if log_dir:
            try:
                files = glob.glob(os.path.join(log_dir, "*.ulg"))
                if files:
                    return max(files, key=os.path.getctime)
            except Exception as e:
                print(f"Error finding log files: {e}")
        return None

    def compress_data(self, data: bytes) -> bytes:
        """Сжимает данные с помощью zlib"""
        return zlib.compress(data)

    async def send_chunk(self, chunk_data: dict) -> bool:
        """Отправляет чанк данных и ждет подтверждения"""
        try:
            encrypted = self.crypto.encrypt_payload(chunk_data)
            await self.websocket.send(encrypted)
            # Ждем ACK в течение 5 секунд
            ack = await asyncio.wait_for(self.websocket.recv(), timeout=5)
            ack_data = json.loads(ack)
            return ack_data.get('type') == 'ack'
        except Exception as e:
            print(f"Error sending chunk: {e}")
            return False

    async def send_file(self, file_path: str) -> bool:
        """Отправляет файл чанками с подтверждением"""
        if not os.path.exists(file_path):
            print(f"File {file_path} does not exist")
            return False

        file_name = os.path.basename(file_path)
        file_size = os.path.getsize(file_path)
        chunks_total = (file_size // self.chunk_size) + 1

        try:
            with open(file_path, 'rb') as f:
                # Отправляем метаданные файла
                metadata = {
                    'name': file_name,
                    'size': file_size,
                    'chunks': chunks_total
                }

                # Отправляем чанки данных
                for chunk_num in range(chunks_total):
                    chunk = f.read(self.chunk_size)
                    compressed = self.compress_data(chunk)
                    
                    chunk_data = {
                        'type': 'file_chunk',
                        'metadata': json.dumps(metadata),
                        'number': chunk_num,
                        'data': base64.b64encode(compressed).decode(),
                        'checksum': zlib.crc32(compressed),
                        'finished': False
                    }
                    chunk_data['finished'] = (chunk_num == chunks_total - 1)
                    if not await self.send_chunk(chunk_data):
                        print(f"Failed to send chunk {chunk_num}")
                        return False
                    print(f"Sent chunk {chunk_num + 1}/{chunks_total}")
                return True

        except Exception as e:
            print(f"Error sending file: {e}")
            return False

    async def connect_and_send(self, file_path: str) -> bool:
        """Устанавливает соединение и отправляет файл"""
        try:
            access_token = self.auth.load_tokens()
            if not self.auth.ensure_tokens_are_valid(access_token):
                print("Invalid tokens, cannot connect", level=1)
                return False
               
            async with websockets.connect(self.server_url, additional_headers={'Authorization': f'Bearer {access_token}'}) as ws:
                self.websocket = ws
                if not await self._initialize_session():
                    print("Failed to initialize secure session")
                    return False
                
                return await self.send_file(file_path)
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    async def monitor_and_send(self):
        """Мониторит директории и отправляет новые файлы"""
        while True:
            try:
                latest_file = self.get_latest_ulog_file()
                if latest_file and latest_file != self.last_sent_file:
                    if self.is_file_complete(latest_file):
                        print(f"Found new complete log file: {latest_file}")
                        if await self.connect_and_send(latest_file):
                            self.last_sent_file = latest_file
                            print("File sent successfully")
                        else:
                            print("Failed to send file")
                    else:
                        print(f"File {latest_file} is still being written or locked, skipping")
                        # Дополнительная информация для отладки
                        print(f"File stats: size={os.path.getsize(latest_file)}, "
                              f"mtime={time.time()-os.path.getmtime(latest_file):.1f}s ago, "
                              f"locked={self.is_file_locked(latest_file)}")
            except Exception as e:
                print(f"Monitoring error: {e}")
            
            await asyncio.sleep(10)

    def is_file_complete(self, file_path: str, 
        check_interval: float = 1.0, 
        max_attempts: int = 3,
        max_age_seconds: float = 5.0) -> bool:
        """
        Комбинированная проверка завершенности файла:
        1. Проверяет, что размер файла стабилен
        2. Проверяет, что файл не изменялся дольше указанного времени
        3. Проверяет, что файл не заблокирован другими процессами
        """
        if not os.path.exists(file_path):
            return False

        # Проверка по времени последнего изменения
        file_age = time.time() - os.path.getmtime(file_path)
        if file_age < max_age_seconds:
            return False

        # Проверка блокировки файла
        if self.is_file_locked(file_path):
            return False

        # Проверка стабильности размера файла
        attempts = 0
        last_size = os.path.getsize(file_path)
        
        while attempts < max_attempts:
            time.sleep(check_interval)
            current_size = os.path.getsize(file_path)
            
            if last_size != current_size:
                return False
                
            last_size = current_size
            attempts += 1

        return True
    
    def is_file_locked(self, file_path: str) -> bool:
        """Проверяет, заблокирован ли файл (открыт другим процессом)"""
        try:
            if os.name == 'posix':  # Linux/Unix
                with open(file_path, 'a') as f:
                    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    return False
            elif os.name == 'nt':  # Windows
                import msvcrt
                with open(file_path, 'a') as f:
                    msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
                    return False
            else:
                # Для других ОС используем только проверку времени изменения
                return False
        except (IOError, BlockingIOError, PermissionError):
            return True
        except Exception:
            # В случае других ошибок считаем файл доступным
            return False

def main():
    handler = UlogHandler()
    
    # Для асинхронного запуска в отдельном потоке
    def run_async():
        asyncio.run(handler.monitor_and_send())
    
    thread = threading.Thread(target=run_async, daemon=True)
    thread.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")

if __name__ == '__main__':
    main()