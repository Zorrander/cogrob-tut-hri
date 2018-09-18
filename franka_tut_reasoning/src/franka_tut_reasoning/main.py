#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Reasoning module.

This module is the core of the system. It interacts with the knowledge base
via a prolog interface defined in a second module in this package to reason
over it.
It can be activated via the GUI or via the senses, i.e., the sensors installed
on the robot.

"""
import rospy

import actionlib
from franka_tut_msgs.msg import *
from franka_tut_msgs.srv import *

from grounding_action_server import GroundingActionServer
from errors import *
from ontology_manager import TaskSemanticManager

from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class Reasoning(object):
    unknown_symbol_publisher = rospy.Publisher('symbol_unknown', String, queue_size=10)

    def __init__(self, kb_interface, planner, action_server):
        self.soundhandle = SoundClient()
        self.kb_interface = kb_interface
        self.planner = planner
        self.action_server = action_server
        self.waiting_human = True

    def speak(self, msg):
        s3 = self.soundhandle.voiceSound(msg)
        s3.play()

    def ending_callback(self, state, result):
        self.speak("I am done")

    def activation_callback(self):
        self.speak("I will!")
        next_move, waiting = self.planner.decide_next_move()
        if waiting:
            print "WAITING FOR HUMAN...."
            self.waiting_human = True
        else:
            self.action_server.send_new_command(next_move)

    def actuator_callback(self, feedback):
        next_move, waiting = self.planner.decide_next_move()
        if waiting:
            print "WAITING FOR HUMAN...."
            self.waiting_human = True
        else:
            self.action_server.send_new_command(next_move)

    def human_callback(self, msg):
        if self.waiting_human:
            self.waiting_human = False
            self.planner.find_next_step()
            next_move, waiting = self.planner.decide_next_move()
            if waiting:
                print "WAITING FOR HUMAN...."
                self.waiting_human = True
            else:
                self.action_server.send_new_command(next_move)

    def callback(self, data):
        """ speech recognition response callback """
        action = data.action
        target = data.target
        try:
            set_skills = self.kb_interface.check_syntax(action, target)
            skill = self.kb_interface.check_semantic(action, target, set_skills)
            steps, constraints = self.kb_interface.retrieve_actions(skill)
            self.kb_interface.procedural_attachment(steps, target)
            print "Constraints : {}".format(constraints)
            self.planner.initialize(steps, constraints)
            client = actionlib.SimpleActionClient('task_action', ExecuteTaskAction)
            client.wait_for_server()
            client.send_goal(ExecuteTaskGoal(order=skill), self.ending_callback, self.activation_callback, self.actuator_callback)
        except (RequestSyntax_Error, TargetUnknown_Error) as e:
            self.speak(e.message)
            Reasoning.unknown_symbol_publisher.publish(String(e.value))
        except Exception as e:
            self.speak(e.message)
