#! /usr/bin/env python

import rospy
import actionlib
from errors import *
from franka_tut_msgs.msg import *

class GroundingActionServer(object):
    _feedback = AddInfoConversationFeedback()
    _result = AddInfoConversationResult()

    def __init__(self, name, kb_interface):
        self.kb_interface = kb_interface
        self._action_name = name
        self.items = []
        self._as = actionlib.SimpleActionServer(self._action_name, AddInfoConversationAction, auto_start = False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()
        print "Grounding server running..."

    def goal_cb(self):
        print "Grounding new symbol"
        goal = self._as.accept_new_goal()
        self.symbol = goal.symbol
        self.server_type = goal.category
        print "I am listening"

    def preempt_cb(self):
        print "Goal preempted"
        self._as.set_preempted()

    def relate_skills(self, msg):
        print "relate skills"
        if (self._as.is_active()):
            try:
                new_skill = msg.list_instruction[0]
                related_skill = msg.list_instruction[1]
                print "matching_attribute {} - {}".format(new_skill, related_skill)
                self.kb_interface.matching_attribute(related_skill.action)
                self.kb_interface.matching_attribute(related_skill.target)
            except (TargetUnknown_Error) as e :
                print(e.message)
            except Exception as e:
                print e
            else:
                print "relate_new_skill({}, {})".format(new_skill, related_skill)
                self.kb_interface.relate_new_skill(new_skill, related_skill)
                msg = "Understood"
                print(msg)

    def new_triple_cb(self, msg):
        '''receives entries from human'''
        if (self._as.is_active()):
            try:
                action = msg.action
                target = msg.target
                if (self.server_type=="actions"):
                    self.kb_interface.check_actions(action) # To be changed
                else:
                    self.kb_interface.matching_attribute(target)
            except (TargetUnknown_Error, FailedToUnderstandAction) as e :
                print(e.message)

            else:
                self.items.append( (action, target) )
                msg = "Understood"
                print(msg)

    def grounding_end_cb(self, msg):
        if (self._as.is_active()):
            self.kb_interface.ground_new_symbol(self.items, self.server_type, self.symbol)
            msg = "Okay got it !"
            print(msg)
            self.items = []
            self._result.anchoring_answer = "Succes"
            self._as.set_succeeded(self._result)
