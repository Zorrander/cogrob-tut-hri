#! /usr/bin/env python

import rospy

from cogrob_tut_reasoning.test_reasoning import Reasonig
from cogrob_tut_reasoning.prolog_interface import PrologInterface
from cogrob_tut_reasoning.grounding_action_server import GroundingActionServer

from cogrob_tut_msgs.msg import *
from cogrob_tut_msgs.srv import *
from std_msgs.msg import String, Empty


if __name__ == '__main__':
    try:
        rospy.init_node('reasoning')

        # initialize publishers
        pl_interface = PrologInterface() # initialize interface with knowrob
        reasoner = Reasonig(pl_interface)
        server = GroundingActionServer('grounding_action', pl_interface)

        # initialize subscriber
        #rospy.Subscriber("/interlocutor", String, reasoner.updateInterlocutor)
        rospy.Subscriber("processed_inputs", Instruction, reasoner.callback)
        rospy.Subscriber("grounding_triple", Instruction, server.new_triple_cb)
        rospy.Subscriber("end_grounding", Empty, server.grounding_end_cb)

        # initialize services (GUI)
        rospy.Service('retrieve_actions', retrieveActions, pl_interface.retrieve_actions)
        rospy.Service('retrieve_targets', retrieveTargets, pl_interface.retrieve_targets)
        rospy.loginfo("Knowledge base services ready.")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
