import struct
from config import*

# basic message object, all messages inherit from this
class Message:
    def pack(self):
        pass
    
    def unpack(self):
        pass

class Empty(Message):
    def __init__(self):
        pass

    def pack(self):
        return b''

class JointPos(Message):
    def __init__(self):
        self.angles = []
        self.__n = 0

    def pack(self):
        data = self.angles
        p = struct.pack(self.__key(), *data)
        return p
    
    def unpack(self, data):
        up = struct.unpack(self.__key(), data)
        self.angles = up

    def resize(self, n):
        self.angles = [0 for i in range(n)]
        self.__n = n
        return self

    def __key(self):
        return '<'+str(self.__n)+'H'