"""
Simulate a robot arm controller hardware.

This node simulates a real robot and provides FollowJointTrajectoryAction action server.

"""

# number of joints can be set in launch file
# @param: joint_nums

import rospy

class ArmSimulator:
    def __init__(self):
        rospy.Subscriber()


if __name__=="__main__":
    rospy.init_node("arm_simulator", anonymous=false)
    