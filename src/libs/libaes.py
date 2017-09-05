#!/usr/bin/env python
import base64
import hashlib
from Crypto import Random
from Crypto.Cipher import AES


class AESCipher(object):

    def __init__(self):
        self.bs = 32
        self.key = None

    def setKey(self, key):
        self.key = hashlib.sha256(key.encode()).digest()

    def encrypt(self, raw):
        raw = self._pad(raw)
        iv = Random.new().read(AES.block_size)
        cipher = AES.new(self.key, AES.MODE_CBC, iv)
        return base64.b64encode(iv + cipher.encrypt(raw))

    def decrypt(self, enc):
        enc = base64.b64decode(enc)
        iv = enc[:AES.block_size]
        cipher = AES.new(self.key, AES.MODE_CBC, iv)
        return self._unpad(cipher.decrypt(enc[AES.block_size:])).decode('ascii')
        # return cipher.decrypt(enc[AES.block_size:])

    def _pad(self, s):
        return s + (self.bs - len(s) % self.bs) * chr(self.bs - len(s) % self.bs)

    @staticmethod
    def _unpad(s):
        return s[:-ord(s[len(s)-1:])]

if __name__=='__main__':
    aes = AESCipher()

    key = bytes(b'10101')
    s = bytes(b'itmo is aaaaaa')

    aes.setKey(key)
    print(s)
    enc = aes.encrypt(s)
    print(enc)
    print(aes.decrypt(enc))
