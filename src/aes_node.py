#!/usr/bin/env python
import rospy
from libs.libaes import AESCipher

from kvant.srv import Aes

class AESServer:

    def __init__(self):
        self.aesServer = rospy.Service('aes_server', Aes, self.handle)
        self.aes = AESCipher()
        rospy.loginfo('AES server started!')
        self.loop()

    def handle(self, request):
        key = request.key
        self.aes.setKey(key)
        data = request.req_data
        response = '';
        if request.mode:
            response = self.aes.encrypt(data)
        else:
            response = self.aes.decrypt(data)
        return response.encode('ascii')

    def loop(self):
        rospy.init_node('aes_node')
        rospy.spin()

if __name__=="__main__":
    AESServer()
