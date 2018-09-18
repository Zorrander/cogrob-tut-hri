#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import String

from franka_tut_msgs.msg import *

def as_doneCb(state, result):
    print "Record motion client inactive"

def as_activeCb():
    print "Record motion client active"

def as_feedbackCb(feedback):
    pass

def send_new_goal(msg, client):
    goal = RecordMotionGoal(msg.data)
    client.send_goal(goal, as_doneCb, as_activeCb, as_feedbackCb)


if __name__ == '__main__':
    try:
        rospy.init_node('record_motion_client', log_level=rospy.DEBUG)

        client = actionlib.SimpleActionClient('record_motion', RecordMotionAction)
        client.wait_for_server()

        print "record_motion client ready"
        rospy.Subscriber('start_recording', String, send_new_goal, client)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
