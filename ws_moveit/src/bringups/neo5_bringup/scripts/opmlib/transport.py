import serial
from config import*

class Transport:
    def __init__(self, port, baudrate=115200):
        self.__serial = serial.Serial(port, baudrate)
    
    def read(self):
        return self.__serial.read()
    
    def write(self, data):
        if DEBUG:
            print(data)
        self.__serial.write(data)
