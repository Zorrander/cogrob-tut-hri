#! /usr/bin/env python

import rospy

from cob_speech_synthesizer.speech_synthesizer import CobSpeechSynthesizer
from std_msgs.msg import String

if __name__ == "__main__":
    try:
        rospy.init_node('speech_synthesizer')

        rospy.loginfo("Starting speech api...")
        
        node = CobSpeechSynthesizer()
        rospy.Subscriber("speech_synthesizer", String, node.callback)

        rospy.spin()
    except :
        pass
