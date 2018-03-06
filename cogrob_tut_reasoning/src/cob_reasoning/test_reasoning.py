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

from grounding_action_server import GroundingActionServer
from errors import *
from prolog_interface import PrologInterface
from rospy_helper import *


class Reasonig(object):

    def __init__(self, pl_interface):
        """ Initializer """
        self.interlocutor = ""
        self.pl_interface = pl_interface

    def callback(self, data):
        """ speech recognition response callback
        @param data: recognized result. 2 attributes : action (e.g. GoTo.) |  target (e.g. Kitchen.)
        """
        # --- extract informations ---
        target = data.target
        action = data.action
        try:
            self.pl_interface.check_actions(action)
            self.pl_interface.check_targets(target)
        except (FailedToUnderstandTarget, FailedToUnderstandAction) as e :
            speak(e.message)
            update_internal_state(e.message)
        else:
            if (action == "Go"):
                #send_command(target)
                msg = "I will!"
                speak(msg)
            else:
                msg = "I am sorry. This capability is not available yet."
                speak(msg)
