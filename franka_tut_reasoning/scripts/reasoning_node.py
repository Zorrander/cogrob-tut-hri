#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from franka_tut_reasoning.ontology_manager import TaskSemanticManager

from franka_tut_reasoning.main import Reasoning
from franka_tut_reasoning.grounding_action_server import GroundingActionServer
from franka_tut_reasoning.task_server import TaskActionServer

from franka_tut_msgs.msg import *
from franka_tut_msgs.srv import *
from std_msgs.msg import Empty

def select_planner_mode():
    mode = rospy.get_param("~mode")
    print "SELECTED MODE IS : {}".format(mode)
    if (mode == "automated"):
        from franka_tut_reasoning.planner import AutomatedPlanner
        result = AutomatedPlanner()
    elif (mode == "interactive"):
        from franka_tut_reasoning.planner import InteractivePlanner
        result = InteractivePlanner()
    elif (mode == "collaborative"):
        from franka_tut_reasoning.planner import CollaborativePlanner
        result = CollaborativePlanner()

    return result

if __name__ == '__main__':
    try:
        #rospy.init_node('reasoning_node', log_level=rospy.DEBUG)
        rospy.init_node('reasoning_node', log_level=rospy.DEBUG)
        knowledge_manager = TaskSemanticManager()
        planner = select_planner_mode()
        server = TaskActionServer('task_action')

        reasoner = Reasoning(knowledge_manager, planner, server)
        grounding_server = GroundingActionServer('grounding_action', knowledge_manager)

        #Initialize subscribers
        rospy.Subscriber("franka_gripper_feedback", Action, server.actuator_cb)
        #rospy.Subscriber("franka_sensor_feedback", Action, server.sensor_feedback_cb)
        rospy.Subscriber("processed_inputs", Instruction, reasoner.callback)
        rospy.Subscriber("grounding_triple", Instruction, grounding_server.new_triple_cb)
        rospy.Subscriber("end_grounding", Empty, grounding_server.grounding_end_cb)
        rospy.Subscriber("grounding_skill_link", Instructions, grounding_server.relate_skills)
        rospy.Subscriber("human_done", Empty, reasoner.human_callback)



        rospy.spin()
    except rospy.ROSInterruptException:
        pass
