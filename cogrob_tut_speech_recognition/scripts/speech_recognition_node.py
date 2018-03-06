#! /usr/bin/env python

import rospy

from cogrob_tut_speech_recognition.speech_recognition import Conversation

from cogrob_tut_msgs.msg import *
from cogrob_tut_msgs.srv import *
from std_msgs.msg import String, Empty

if __name__ == '__main__':
    try:

        # initialize publisher
        rospy.init_node('speech_recognition')
        processed_input_publisher = rospy.Publisher('processed_inputs', Instruction, queue_size=10)
        publisher_ss = rospy.Publisher('speech_synthesizer', String, queue_size=10)
        pub_grounding_triple = rospy.Publisher("grounding_triple", Instruction, queue_size=10)
        pub_end_grounding = rospy.Publisher("end_grounding", Empty, queue_size=10)
        listener = Conversation(processed_input_publisher, publisher_ss, pub_grounding_triple, pub_end_grounding)

        raw_input_subscriber = rospy.Subscriber('speech_to_text', String, listener.sr_processing)
        symbol_unknown_subscriber = rospy.Subscriber('symbol_unknown', String, listener.symbol_unknown)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
