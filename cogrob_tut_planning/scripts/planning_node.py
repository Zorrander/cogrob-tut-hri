#! /usr/bin/env python

import rospy

from cogrob_tut_planning.controller import Controller
from std_msgs.msg import String

if __name__ == '__main__':
    try:
        rospy.init_node('cogrob_tut_controller')

        rospy.loginfo("Starting the controler for the robot...")

        controller = Controller()
        rospy.Subscriber("action_commands", String, controller.callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
