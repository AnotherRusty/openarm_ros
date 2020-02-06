from transport import Transport
from protocol import *
import threading
from config import*
from messages import*
from time import time as now, sleep
import yaml
import os


class RobotStatus:
    def __init__(self, num_joints):
        self.current_joint_positions = JointPos().resize(num_joints)

class Robot:
    def __init__(self):
        self.name = None
        self.__protocol = Protocol()
        self.__shutdown = False
        self.__initialized = False
        self.__configured = False
        self.__waiting_for = None

    def configure(self, name, port, baudrate):
        self.name = name
        home_path = os.getenv('HOME')
        config_file = home_path+'/openarm_ros/ws_moveit/src/bringups/victor5_bringup/scripts/config/'+self.name+'.yaml'
        with open(config_file) as f:
            conf = yaml.safe_load(f)
            f.close()
        self.__port = port
        self.__baudrate = baudrate
        self.__num_joints = len(conf['Joints'])
        self.__configured = True

    def initialize(self):
        if not self.__configured:
            raise RuntimeError('robot has not been configured.')
        try:
            self.__transport = Transport(self.__port, self.__baudrate)
        except:
            print("serial initialization failed")
            raise RuntimeError("Robot init failed. Check configuration.")
        self.__proc = threading.Thread(name='robot process', target=self.__run)
        self.__proc.setDaemon(True)
        self.robot_status = RobotStatus(self.__num_joints)
        self.__initialized = True

    def start(self):
        if not self.__initialized:
            raise RuntimeError('robot has not been initialized.')
        self.__proc.start()

    def shutdown(self):
        self.__shutdown = True  # set shutdown flag

    def get_joint_angles(self):
        self.__set_wait(MsgId.RET_JOINT_POSITIONS)
        self.__transport.write(self.__protocol.encode(MsgId.GET_JOINT_POSITIONS, Empty()))
        if self.__wait():
            return self.robot_status.current_joint_positions.angles
        else:
            return False

    def set_joint_angles(self, angles):
        print("setting joint angles at ", angles)
        msg = JointPos().resize(self.__num_joints)
        for i in range(self.__num_joints):
            msg.angles[i] = angles[i]
        self.__transport.write(self.__protocol.encode(MsgId.SET_JOINT_POSITIONS, msg))

    def __run(self):
        while not self.__shutdown:
            c = self.__transport.read()
            # print(c)
            if self.__protocol.parse(c):
                self.__process_message(self.__protocol.get_message()) 

    def __process_message(self, msg_id):
        if msg_id == MsgId.MOVE_DONE:
            pass
        elif msg_id == MsgId.RET_JOINT_POSITIONS:
            self.__protocol.decode(msg_id, self.robot_status.current_joint_positions)
            if self.__waiting_for == MsgId.RET_JOINT_POSITIONS:
                self.__waiting_for = None

    def __set_wait(self, msg_id):
        self.__waiting_for = msg_id
    
    def __wait(self, timeout=0.5):
        _timeout = now()+timeout
        while now() < _timeout:
            if self.__waiting_for is None:
                return True
        return False
            