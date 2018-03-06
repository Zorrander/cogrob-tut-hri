#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Speech synthesizer module.

This module uses a feature provided by the Care-O-Bot to be able to speak out
sentences. The /sound/mode parameter defines which engine is used to do so.

Todo:
    * Translation operations could be useful.

"""

import rospy

import roslib
roslib.load_manifest('cob_script_server')

from simple_script_server import *

from std_msgs.msg import String

class CobSpeechSynthesizer():

    """Class exploiting the speech capabilities of the Care-O-Bot"""

    def __init__(self):
        self._sss = simple_script_server()
        self._sss.say("sound", ["I am listening to you"])

    def callback(self, data):
        msg = data.data
        print ("Saying : %s" % msg)
        self._sss.say("sound", [msg])
