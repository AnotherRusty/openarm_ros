''' protocol for transmission between the embedded system and the pc '''

from config import* 
from utils import bytes_to_int as int
from utils import int_to_bytes as bytes


__all__ = ["MsgId", "Protocol"]

class MsgId:
    DUMMY = 0
    SET_JOINT_POSITIONS = 1
    GET_JOINT_POSITIONS = 2

    MOVE_DONE = 100
    RET_JOINT_POSITIONS = 101

class ParseState:
    WAITING_FOR_HEAD = 1
    WAITING_FOR_ID = 2
    WAITING_FOR_LEN = 3
    WAITING_FOR_DATA = 4
    CHECKSUM = 5

class Protocol:
    def __init__(self):
        self.__head = '0x5a'
        self.__data = b''
        self.__msg_id = None
        self.__msg_len = 0
        self.__checksum = 0
        self.__parse_state = ParseState.WAITING_FOR_HEAD
        self.__new_message = None

    def get_message(self):
        return self.__new_message

    def parse(self, c):
        if len(c) == 0: # no data received
            return False
        if DEBUG:
            # print(self.__parse_state)
            print(repr(c))
        if self.__parse_state == ParseState.WAITING_FOR_HEAD:
            # reset msg 
            self.__msg_id = None
            self.__checksum = 0
            self.__data = b''

            if c == bytes(HEAD):
                if DEBUG:
                    print('got head')
                self.__checksum += ord(c)
                self.__parse_state += 1
            return False
        if self.__parse_state == ParseState.WAITING_FOR_ID:
            if DEBUG:
                print('got id ', ord(c))
            self.__msg_id = ord(c)
            self.__checksum += ord(c)
            self.__parse_state += 1
            return False
        if self.__parse_state == ParseState.WAITING_FOR_LEN:
            if DEBUG:
                print('got len ', ord(c))
            self.__msg_len = ord(c)
            self.__checksum += ord(c)
            self.__parse_state += 1
            return False
        if self.__parse_state == ParseState.WAITING_FOR_DATA:
            if self.__msg_len > 0:
                self.__data += c
                self.__checksum += ord(c)
                self.__msg_len -= 1
            else:
                if DEBUG:
                    print('got complete data')
                self.__parse_state = ParseState.CHECKSUM
        if self.__parse_state == ParseState.CHECKSUM:
            if DEBUG:
                print(ord(c), "  ", self.__checksum&0xff)
            if ord(c) == self.__checksum&0xff:
                if DEBUG:
                    print('checksum ok')
                self.__new_message =  self.__msg_id
                self.__parse_state = ParseState.WAITING_FOR_HEAD
                return True
            else:
                if DEBUG:
                    print('checksum error, message discarded')
                self.__parse_state = ParseState.WAITING_FOR_HEAD
                return False       

    def encode(self, msg_id, msg):
        ''' encodes a message
            returns a bytes str ready to send
            
            @param msg_id: message id
            @param msg: a message object
            ---
            @return b_msg: bytes str
        '''
        head = bytes(HEAD)
        msg_id = bytes(msg_id)
        data = msg.pack()
        msg_len = bytes(len(data)) 
        lb_msg = head + msg_id + msg_len + data
        checksum = 0
        for i in range(len(lb_msg)):
            checksum += int(lb_msg[i])
        checksum = checksum&0xff
        b_msg = lb_msg + bytes(checksum)
        return b_msg
    
    def decode(self, msg_id, msg):
        ''' decodes a in-coming data into
            a message

            @param msg_id: message id
            @param msg: message object
            ---
            @return : bool
        '''
        if (msg_id != self.__msg_id):
            if DEBUG:
                print("unpack error")
            return False
        msg.unpack(self.__data)
        self.__clear_message()
        return True
    
    def __clear_message(self):
        self.__new_message = None

        