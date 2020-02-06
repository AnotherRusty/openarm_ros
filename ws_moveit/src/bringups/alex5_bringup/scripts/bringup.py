#!/usr/bin/env python

import rospy
from opmlib.robot import Robot
from time import sleep, time
from sensor_msgs.msg import JointState
from math import pi


NAME = 'alex5'
PORT = '/dev/ttyACM0'
BAUDRATE = 9600

NOMINAL_POSITION = 90
NUM_JOINTS = 5

angles = [NOMINAL_POSITION for i in range(NUM_JOINTS)]
last_time = time()
new_command_flag = False

def move_callback(joint_state):
    rospy.loginfo('got new move command')
    joint_positions = joint_state.position
    
    for i in range(len(joint_positions)):
        new_position = joint_positions[i]
        angle = NOMINAL_POSITION - (new_position / pi * 180)
        angles[i] = int(angle)
    print(angles)

    global new_command_flag, last_time
    new_command_flag = True
    last_time = time()

# initialize robot
robot = Robot()
robot.configure(name=NAME, port=PORT, baudrate=BAUDRATE)
robot.initialize()
robot.start()
sleep(3)

# initialize ros node
rospy.init_node('bringup', anonymous=False)
rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, move_callback)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if new_command_flag:
        if (time()-last_time) > 1.0:
            rospy.loginfo('send_command %d, %d, %d, %d, %d', angles[0], angles[1], angles[2], angles[3], angles[4])
            robot.set_joint_angles(angles)
            new_command_flag = False
    rate.sleep()
