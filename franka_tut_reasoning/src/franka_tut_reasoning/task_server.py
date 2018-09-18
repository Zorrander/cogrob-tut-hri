#! /usr/bin/env python

import rospy

import actionlib
from franka_tut_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommand

class TaskActionServer(object):
    feedback = ExecuteTaskFeedback()
    result = ExecuteTaskResult()
    pose = Pose()
    motion_name = String()
    gripper_command = GripperCommand()
    joint_state = JointState()
    pub_cartesian_motion = rospy.Publisher('move_arm', Pose, queue_size=10)
    pub_gripper_motion = rospy.Publisher('move_gripper', GripperCommand, queue_size=10)
    pub_joint_action = rospy.Publisher('move_joints', JointState, queue_size=10)
    pub_motion = rospy.Publisher('/motion_cmd', String, queue_size=10)

    def __init__(self, name):
        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(self.action_name, ExecuteTaskAction, auto_start = False)
        self.action_server.register_goal_callback(self.goal_cb)
        self.action_server.register_preempt_callback(self.preempt_cb)
        self.action_server.start()
        self.map_step_arg = {
                'CartesianMotion': self.send_cartesian_motion,
                'GripperMotion': self.send_gripper_motion,
                'JointAction': self.send_joint_action
        }

    def goal_cb(self):
        self.goal = self.action_server.accept_new_goal().order

    def preempt_cb(self):
        self.action_server.set_preempted()

    def actuator_cb(self, msg):
        if (self.action_server.is_active()):
            # publish the feedback
            self.feedback.robot_feedback.action_name = msg.action_name
            self.feedback.robot_feedback.status = msg.status
            self.action_server.publish_feedback(self.feedback)
            print "published feedback {}".format(self.feedback)

    def sensor_feedback_cb(self, msg):
        if (self.action_server.is_active()):
            self.feedback.human_feedback.action_name = msg.action_name
            self.feedback.human_feedback.status = msg.status
            self.action_server.publish_feedback(self.feedback) # publish the feedback
            if msg.status == 0:
                self.handle_human_done()

    def send_new_command(self, next_move):
        """Just look for the next step"""
        if next_move:
            self.send_motion(next_move)
        else:
            self.result.result = "Succes"
            self.action_server.set_succeeded(self.result)


    @classmethod
    def send_motion(cls, motion_name):
        cls.motion_name.data = motion_name
        print "sending {}".format(cls.motion_name)
        cls.pub_motion.publish(cls.motion_name)

    @classmethod
    def send_cartesian_motion(cls, poses):
        cls.pose.position.x = float(poses[0])
        cls.pose.position.y = float(poses[1])
        cls.pose.position.z = float(poses[2])
        print "sending {}".format(cls.pose)
        cls.pub_cartesian_motion.publish(cls.pose)

    @classmethod
    def send_gripper_motion(cls, width):
        cls.gripper_command.position = float(width)
        print "sending {}".format(cls.gripper_command)
        cls.pub_gripper_motion.publish(cls.gripper_command)

    @classmethod
    def send_joint_action(cls, angle):
        cls.joint_state.name.append("end_effector")
        cls.joint_state.position.append(float(angle))
        print "sending {}".format(cls.joint_state)
        cls.pub_joint_action.publish(cls.joint_state)
