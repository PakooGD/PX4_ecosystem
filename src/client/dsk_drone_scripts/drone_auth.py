import keyring
import time
import uuid
import os
import jwt
import requests
import socket
from typing import Optional, Tuple

# ===== Configuration =====
HTTP_URL = os.getenv('DRONE_API_URL', 'http://localhost:5000/api')
TOKEN_CHECK_INTERVAL = 10
KEYRING_SERVICE = "drone_auth_service"
# =========================

class AuthGateway:
    def __init__(self):
        # Загружаем токены
        self._load_tokens()

    def _get_or_create_drone_id(self) -> str:
        """Получает или создает и сохраняет ID дрона в keyring"""
        try:
            # Пытаемся получить сохраненный ID
            drone_id = keyring.get_password(KEYRING_SERVICE, "drone_id")
            
            if not drone_id:
                # Генерируем новый UUID если ID нет
                drone_id = str(uuid.uuid4())
                keyring.set_password(KEYRING_SERVICE, "drone_id", drone_id)
                print(f"Generated new Drone ID: {drone_id}")
            else:
                print(f"Loaded existing Drone ID: {drone_id}")
                
            return drone_id
        except Exception as e:
            print(f"Error handling drone ID: {e}")
            return str(uuid.uuid4())  # Fallback

    def _load_tokens(self) -> Tuple[Optional[str], Optional[str]]:
        """Загружает токены из secure storage"""
        try:
            access_token = keyring.get_password(KEYRING_SERVICE, "access_token")
            refresh_token = keyring.get_password(KEYRING_SERVICE, "refresh_token")
            return access_token, refresh_token
        except Exception as e:
            print(f"Error loading tokens from keyring: {e}")
            return None, None

    def _save_tokens(self, access_token: str, refresh_token: str) -> bool:
        """Сохраняет токены в secure storage"""
        try:
            keyring.set_password(KEYRING_SERVICE, "access_token", access_token)
            keyring.set_password(KEYRING_SERVICE, "refresh_token", refresh_token)
            return True
        except Exception as e:
            print(f"Error saving tokens to keyring: {e}")
            return False

    def _delete_all_credentials(self):
        """Удаляет все сохраненные данные из keyring"""
        try:
            # Удаляем токены
            keyring.delete_password(KEYRING_SERVICE, "access_token")
            keyring.delete_password(KEYRING_SERVICE, "refresh_token")
            # Удаляем ID дрона (опционально)
            keyring.delete_password(KEYRING_SERVICE, "drone_id")
            print("Successfully deleted")
        except Exception as e:
            print(f"Error deleting credentials: {e}")

    def _get_token_expiration(self, token: str) -> float:
        """Возвращает время истечения токена"""
        try:
            decoded = jwt.decode(token, options={"verify_signature": False})
            return decoded.get('exp', 0)
        except Exception as e:
            print(f"Token validation error: {e}")
            return 0

    def _is_token_expired(self, token: str) -> bool:
        """Проверяет, истек ли токен"""
        if not token:
            return True
        exp = self._get_token_expiration(token)
        return time.time() >= exp

    def _is_token_expiring_soon(self, token: str, threshold_seconds: int = 60) -> bool:
        """Проверяет, истекает ли токен скоро"""
        if not token:
            return True
        exp = self._get_token_expiration(token)
        return 0 < exp - time.time() < threshold_seconds

    def get_local_ip(self) -> str:
        """Получает локальный IP адрес"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "unknown"

    def authenticate(self,retry_delay = 1) -> Tuple[Optional[str], Optional[str]]:
        """Аутентификация и получение новых токенов"""
        payload = {
            'drone_id': self._get_or_create_drone_id(),
            'ip_address': self.get_local_ip()
        }
        
        while True:
            try:
                response = requests.post(
                    f"{HTTP_URL}/auth",
                    json=payload,
                    timeout=10
                )
                if response.status_code == 200:
                    data = response.json()
                    access_token = data.get('accessToken')
                    refresh_token = data.get('refreshToken')

                    if not all([access_token, refresh_token]):
                        print("Invalid response from server - missing tokens or key")
                        continue

                    if self._save_tokens(access_token, refresh_token):
                        print("Authentication successful")
                        return access_token, refresh_token
                    else:
                        print("Failed to save tokens")
                print(f"Authentication failed: {response.status_code} - {response.text}. Try again in {retry_delay} seconds...")
            except Exception as e:
                print(f"Auth error: {e}. Reconnecting in {retry_delay} seconds...")    
            time.sleep(retry_delay)
            retry_delay = min(retry_delay * 2, 30)

    def refresh_tokens(self, refresh_token: str) -> Tuple[Optional[str], Optional[str]]:
        """Обновление токенов с использованием refresh token"""
        try:
            response = requests.post(
                f"{HTTP_URL}/refresh",
                json={'refresh_token': refresh_token},
                timeout=10
            )
            if response.status_code == 200:
                data = response.json()
                new_access_token = data.get('accessToken')
                new_refresh_token = data.get('refreshToken')
                
                if self._save_tokens(new_access_token, new_refresh_token):
                    print("Tokens refreshed successfully")
                    return new_access_token, new_refresh_token
                else:
                    print("Failed to save refreshed tokens")
            print(f"Refresh failed: {response.status_code} - {response.text}")
        except Exception as e:
            print(f"Refresh error: {e}")
        return None, None

    def check_and_renew_tokens(self):
        """Проверяет и обновляет токены при необходимости"""
        access_token, refresh_token = self._load_tokens()
        
        if not access_token or self._is_token_expired(access_token) or self._is_token_expiring_soon(access_token):
            print("Access token needs renewal")
            if not refresh_token or self._is_token_expired(refresh_token) or self._is_token_expiring_soon(refresh_token, 300):
                print("Refresh token expired or expiring soon, authenticating...")
                self.authenticate()
            else:
                self.refresh_tokens(refresh_token)
        else:
            print("Tokens are valid")

def main():
    auth = AuthGateway()

    try:
        while True:
            auth.check_and_renew_tokens()
            time.sleep(TOKEN_CHECK_INTERVAL)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        auth._delete_all_credentials() # for dev needs only
        pass

if __name__ == '__main__':
    main()