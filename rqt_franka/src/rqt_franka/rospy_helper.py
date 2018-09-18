import rospy
from franka_tut_msgs.srv import *
from franka_tut_msgs.msg import *
processed_input_publisher = rospy.Publisher('processed_inputs', Instruction, queue_size=10)

def retrieve_tasks_client(label):
    rospy.wait_for_service('retrieve_tasks')
    try:
        retrieve_tasks = rospy.ServiceProxy('retrieve_tasks', TaskInfo)
        list_tasks = retrieve_tasks(label)
        return list_tasks.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


class Timer(rospy.Timer):

    def __init__(self, frequency, callback):
        super(Timer, self).__init__(rospy.Duration(frequency), callback)


def create_instruction_msg(action, target):
    msg = Instruction()
    msg.action = action
    msg.target = target
    processed_input_publisher.publish(msg)
