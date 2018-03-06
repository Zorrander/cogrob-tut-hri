#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Planning module.

This module is meant to handle the physical tasks on the robot. With the Care-O-Bot
it means using a simble_scipt_server object to send command to the simulated or real
robot.

For now it focuses on the move command for the base. It requires to be able to
ask for coordinates and check the parameter server for the presence or not of informations
about the target. These informations will be under the namespace "/base"

Todo:
    * More actions than just move_base
    * Use the full potential of actions by handling the ongoing feedback.
    * For a set of actions being able to decompose them into subset

"""
import rospy

import roslib
roslib.load_manifest('cob_script_server')
from simple_script_server import *

class Controller():

    """Class exploiting the physical capabilities of the Care-O-Bot"""

    def __init__(self):
        self._sss = sss = simple_script_server()
        #self.sss.init("head")
        sss.init("torso")
        sss.init("arm_left")
        sss.init("arm_right")
        sss.init("gripper_left")
        sss.init("gripper_right")
        sss.init("base")

    def callback(self, data):
        target = data.data
        move = self._sss.move("base", target)
        #move.wait()
        #result = move.get_state()
