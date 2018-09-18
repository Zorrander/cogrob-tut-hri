#! /usr/bin/env python

import unittest
import rosunit

from franka_tut_reasoning.rdf_manager import RdfManager
from franka_tut_reasoning.entity_manager import EntityManager
from franka_tut_reasoning.planner import Planner

class RdfTest(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(RdfTest, self).__init__(*args, **kwargs)
        self.rdf = RdfManager("http://webprotege.stanford.edu/cogrob.tut#", "/home/anglerau/Franka/catkin_ws/src/cog-rob-tut/franka_tut_reasoning/owl/")
        self.rdf.load("root-ontology.owl")
        self.entity_manager =  EntityManager("http://webprotege.stanford.edu/cogrob.tut#")

    def find_new_objects(self):
        if (self.rdf.find("Sensor") and self.rdf.find("sensor01")):
            self.rdf.print_info("Sensor")
            self.rdf.print_info("cranfieldStep_001")
            return True
        else:
            return False

    def runTest(self):
        new_sensor_class_info = self.entity_manager.create_entity("Sensor", "Object")
        new_sensor_ind_info = self.entity_manager.create_individual("sensor01", "Sensor")

        self.rdf.add_to_graph(new_sensor_class_info)
        self.rdf.add_to_graph(new_sensor_ind_info)

        self.assertTrue(self.find_new_objects())

class PlanInstance(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(PlanInstance, self).__init__(*args, **kwargs)
        self.rdf = RdfManager("http://webprotege.stanford.edu/cogrob.tut#", "/home/anglerau/Franka/catkin_ws/src/cog-rob-tut/franka_tut_reasoning/owl/")
        self.rdf.load("root-ontology.owl")
        self.entity_manager =  EntityManager("http://webprotege.stanford.edu/cogrob.tut#")
        self.action_number = 0

    def print_actions(self):
        print "ArmAction individual : "
        print self.rdf.getAllIndividualFromClass("ArmAction")
        print "GripperAction individual :"
        print self.rdf.getAllIndividualFromClass("GripperAction")
        steps = self.rdf.getAllIndividualFromClass("CranfieldStepsAction")
        for step in steps:
            self.rdf.print_info(step)

    def runTest(self):
        steps = self.rdf.getTaskSteps("Cranfield_001")
        constraints = self.rdf.getTaskConstraints("Cranfield_001")
        planner = Planner()
        planner.initializeTaskGraph(steps)
        planner.updateTaskGraph(constraints)
        plan = []
        done = False
        while (not done):
            next_move = planner.decideNextMove()
            if (not next_move):
                done = True
            else:
                class_constraints = []
                ind_types = self.rdf.get_types(next_move)
                for a_type in ind_types:
                        class_constraints.append(self.rdf.classDefinesConstraints(a_type))
                new_ind = self.entity_manager.build_individuals( next_move, class_constraints, self.rdf.generateIndvidualId("ComplexAction") )
                for i in new_ind:
                    self.rdf.add_to_graph(i)
                plan.append(next_move)

        self.print_actions()
        self.assertTrue(True)




if __name__ == '__main__':
    #rosunit.unitrun('franka_tut_reasoning', 'test_manager_rdf', RdfTest)
    rosunit.unitrun('franka_tut_reasoning', 'test_plan_instance', PlanInstance)
