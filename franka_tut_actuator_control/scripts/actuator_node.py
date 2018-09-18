#! /usr/bin/env python

import rospy
import sys
import copy
import rosbag
import moveit_commander

from math import pi

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
import moveit_msgs.msg
import geometry_msgs.msg

from franka_tut_msgs.msg import *
from franka_tut_actuator_control.record_motion import RecordMotionServer
from franka_tut_actuator_control.arm_cmd import PandaArm


if __name__ == '__main__':
    try:
        rospy.init_node('actuator_node', log_level=rospy.DEBUG)

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("panda_arm")

        panda_arm = PandaArm(group)
        recording_server = RecordMotionServer('record_motion', group)

        rospy.Subscriber('/motion_cmd', String, panda_arm.motion_command)
        rospy.Subscriber('end_record_motion', Empty, recording_server.stop_recording_callback)
        rospy.Subscriber('/joint_states', JointState, recording_server.listen_joint_space_feed)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
