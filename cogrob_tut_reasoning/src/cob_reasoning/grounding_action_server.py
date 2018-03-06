#! /usr/bin/env python

import rospy
import actionlib
from errors import *
from cogrob_tut_msgs.msg import *
from rospy_helper import *
from prolog_interface import PrologInterface

class GroundingActionServer(object):
    _feedback = AddInfoConversationFeedback()
    _result = AddInfoConversationResult()

    def __init__(self, name, pl_interface):
        self.pl_interface = pl_interface
        self._action_name = name
        self.items = []
        self._as = actionlib.SimpleActionServer(self._action_name, AddInfoConversationAction, auto_start = False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()
        print "Task server running..."

    def goal_cb(self):
        print "Grounding new symbol"
        goal = self._as.accept_new_goal()
        self.symbol = goal.symbol
        self.server_type = goal.category
        msg = "I am listening"
        speak(msg)

    def preempt_cb(self):
        print "Goal preempted"
        self._as.set_preempted()


    def new_triple_cb(self, msg):
        '''receives entries from human'''
        if (self._as.is_active()):
            try:
                action = msg.action
                target = msg.target
                if (self.server_type=="actions"):
                    self.pl_interface.check_actions(action)
                else:
                    self.pl_interface.check_targets(target)
            except (FailedToUnderstandTarget, FailedToUnderstandAction) as e :
                speak(e.message)

            else:
                self.items.append( (action, target) )
                msg = "Understood"
                speak(msg)

    def grounding_end_cb(self, msg):
        if (self._as.is_active()):
            self.pl_interface.ground_new_symbol(self.items, self.server_type, self.symbol)
            msg = "Okay got it !"
            speak(msg)
            self.items = []
            self._result.anchoring_answer = "Succes"
            self._as.set_succeeded(self._result)
