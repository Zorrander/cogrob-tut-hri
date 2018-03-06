import rospy

from std_msgs.msg import String

from cogrob_tut_msgs.msg import *

publisher_ss = rospy.Publisher('speech_synthesizer', String, queue_size=10)
publisher_commands = rospy.Publisher('action_commands', String, queue_size=10)
publisher_sm_feedback = rospy.Publisher('symbol_unknown', String, queue_size=10)

def speak(msg):
    publisher_ss.publish(msg)

def update_internal_state(msg):
    publisher_sm_feedback.publish(msg)

def send_command(target):
    publisher_commands.publish(target)
