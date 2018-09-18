#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from franka_tut_msgs.msg import *
from franka_tut_nlp.speech_recognition import Conversation


if __name__ == '__main__':
    try:
        rospy.init_node('speech_recognition')

        listener = Conversation()

        raw_input_subscriber = rospy.Subscriber('speech_to_text', String, listener.sr_processing)
        symbol_unknown_subscriber = rospy.Subscriber('symbol_unknown', String, listener.symbol_unknown)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
