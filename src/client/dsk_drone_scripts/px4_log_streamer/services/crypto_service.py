from Crypto.Cipher import AES, PKCS1_OAEP
from Crypto.PublicKey import RSA
from Crypto.Util.Padding import pad, unpad
import base64
import os
import json

class CryptoHandler:
    def __init__(self):
        self.session_key = os.urandom(32)
        self.session_iv = os.urandom(16)
        self.server_public_key = None
    
    def set_server_key(self, public_key_pem: str):
        """Устанавливает публичный ключ сервера"""
        try:
            self.server_public_key = RSA.import_key(public_key_pem)
        except Exception as e:
            raise ValueError(f"Invalid public key: {str(e)}")
    
    def encrypt_session_key(self) -> dict:
        """Шифрует сессионный ключ для отправки на сервер"""
        if not self.server_public_key:
            raise ValueError("Server public key not set")
        
        cipher = PKCS1_OAEP.new(self.server_public_key)
        return {
            'key': base64.b64encode(cipher.encrypt(self.session_key)).decode(),
            'iv': base64.b64encode(cipher.encrypt(self.session_iv)).decode()
        }
    
    def encrypt_payload(self, data: dict) -> str:
        """Шифрует данные сессионным ключом"""
        cipher = AES.new(self.session_key, AES.MODE_CBC, self.session_iv)
        ct_bytes = cipher.encrypt(pad(json.dumps(data).encode(), AES.block_size))
        return json.dumps({
            'iv': base64.b64encode(self.session_iv).decode(),
            'ciphertext': base64.b64encode(ct_bytes).decode()
        })