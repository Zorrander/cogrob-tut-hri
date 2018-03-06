import actionlib
from std_msgs.msg import String
from cogrob_tut_msgs.msg import *


def create_instruction_msg(action, target):
    msg = Instruction()
    msg.action = action
    msg.target = target
    return msg


def create_grounding_client(goal, doneCb, activeCb, feedbackCb):
    client = actionlib.SimpleActionClient('grounding_action', AddInfoConversationAction)
    print ("Waiting to connect to the grounding server")
    client.wait_for_server()
    client.send_goal(goal, doneCb, activeCb, feedbackCb)
