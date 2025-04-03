import keyring
import time
import jwt

# Security configuration
KEYRING_SERVICE = "drone_auth_service"

class AuthHandler:  
    def load_tokens(self):
            """Загружает токены из keyring"""
            try:
                return keyring.get_password(KEYRING_SERVICE, "access_token")
            except Exception as e:
                print(f"Error loading tokens from keyring: {e}")
                return None

    def _is_token_expired(self, token: str) -> bool:
            """Проверяет, истек ли срок действия токена."""
            try:
                decoded = jwt.decode(token, options={"verify_signature": False})
                expiration_time = decoded.get('exp')
                if expiration_time is None:
                    return True
                return time.time() >= expiration_time
            except Exception as e:
                print(f"Error decoding token: {e}")
                return True       

    def ensure_tokens_are_valid(self, token) -> bool:
                """Проверяет актуальность токенов."""
                if not token or self._is_token_expired(token):
                    print("Access token is expired or missing. Wait for refreshing...")
                    return False  
                return True  
