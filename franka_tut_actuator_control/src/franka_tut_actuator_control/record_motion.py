
import rospy
import rosbag
import actionlib
from franka_tut_msgs.msg import RecordMotionAction, RecordMotionFeedback, RecordMotionResult, Gripper, Arm

'''
TODO : Should only listen to robot state when activated
'''
class RecordMotionServer(object):
    feedback = RecordMotionFeedback()

    def __init__(self, name, group):
        self.action_name = name
        self.group = group
        self.action_server = actionlib.SimpleActionServer(self.action_name, RecordMotionAction, auto_start = False)
        self.action_server.register_goal_callback(self.goal_cb)
        self.action_server.register_preempt_callback(self.preempt_cb)
        self.action_server.start()
        self.arm_state = Arm()
        self.gripper_state = Gripper()
        self.previous_arm_state = []
        self.previous_gripper_state = [0.0, 0,0]

    def listen_joint_space_feed(self, msg):
        if not (self.previous_arm_state == msg.position[:-2]):
            self.previous_arm_state = msg.position[:-2]
            self.arm_state.joint_1 = msg.position[0]
            self.arm_state.joint_2 = msg.position[1]
            self.arm_state.joint_3 = msg.position[2]
            self.arm_state.joint_4 = msg.position[3]
            self.arm_state.joint_5 = msg.position[4]
            self.arm_state.joint_6 = msg.position[5]
            self.arm_state.joint_7 = msg.position[6]

        if not (self.previous_gripper_state[0] == msg.position[-2]):
            self.gripper_state.finger1 = msg.position[-2]
        if not (self.previous_gripper_state[1] == msg.position[-1]):
            self.gripper_state.finger2 = msg.position[-1]

    def goal_cb(self):
        self.goal = self.action_server.accept_new_goal().name
        print "ACCEPTED GOAL : {}".format(self.goal)
        path = '/home/admin-franka/catkin_ws/src/cog-rob-tut/franka_tut_actuator_control/src/franka_tut_actuator_control/bagfiles/'
        filename = self.goal
        extension = '.bag'
        self.bag = rosbag.Bag((path + filename + extension), 'w')
        self.has_listener = True
        previous_arm_entry = Arm()
        previous_gripper_entry = Gripper()
        print ("Starting to write")
        while self.has_listener:
            if not (previous_arm_entry == self.arm_state):
                self.bag.write('arm_motion', self.arm_state)
                previous_arm_entry = self.arm_state
            if not (previous_gripper_entry == self.gripper_state):
                self.bag.write('gripper_motion', self.gripper_state)
                previous_gripper_entry = self.gripper_state

    def preempt_cb(self):
        if self.action_server.is_active():
            self.action_server.set_preempted()

    def stop_recording_callback(self, msg):
        self.has_listener = False
        self.action_server.set_succeeded(RecordMotionResult())
        self.bag.close()
        print "============ STOP RECORDING ================"
