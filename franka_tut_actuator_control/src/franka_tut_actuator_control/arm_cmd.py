#! /usr/bin/env python

import rospy
import sys
import copy
import rosbag
import actionlib

from math import pi

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
from franka_gripper.msg import MoveAction, MoveGoal
import moveit_msgs.msg
import geometry_msgs.msg

from franka_tut_msgs.msg import *

class PandaArm():

    _feedback_publisher = rospy.Publisher('franka_gripper_feedback', Action, queue_size=10)
    _bags_path = '/home/admin-franka/catkin_ws/src/cog-rob-tut/franka_tut_actuator_control/src/franka_tut_actuator_control/bagfiles/'
    _extension = '.bag'

    def __init__(self, group):
        '''Initialize arm control interface'''
        self.joint_goal = [0.0]*7
        self.set_joint_goal([0, -pi/4, 0, -pi/2, 0, pi/3, 0])
        self.group = group
        self.group.go(self.joint_goal, wait=True)
        self.group.stop()

        self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.gripper_client.wait_for_server()
        print ("* Ready to operate Panda arm *")

    def set_joint_goal(self, values):
        self.joint_goal[0] = values[0]
        self.joint_goal[1] = values[1]
        self.joint_goal[2] = values[2]
        self.joint_goal[3] = values[3]
        self.joint_goal[4] = values[4]
        self.joint_goal[5] = values[5]
        self.joint_goal[6] = values[6]

    def motion_command(self, name):
        filename = name.data
        bag = rosbag.Bag(self._bags_path + filename + self._extension)
        for topic, msg, t in bag.read_messages():
            if topic == 'arm_motion':
                self.set_joint_goal([msg.joint_1, msg.joint_2, msg.joint_3,
                                        msg.joint_4, msg.joint_5, msg.joint_6, msg.joint_7])
                self.group.go(self.joint_goal, wait=True)
                self.group.stop()
            elif topic == 'gripper_motion':
                self.gripper_action_client(msg.finger1 + msg.finger2)
        bag.close()
        self._feedback_publisher.publish(Action(status = 0))

    def gripper_action_client(self, width, speed=0.1):
        '''moves to a target width with the defined speed.'''
        self.gripper_client.send_goal(MoveGoal(width, speed))
        self.gripper_client.wait_for_result()
        self._feedback_publisher.publish(Action(status=0))
        return self.gripper_client.get_result()
