#! /usr/bin/env python

import unittest
import rosunit
from franka_tut_reasoning.rdf_manager import RdfManager
from franka_tut_reasoning.planner import Planner

class MakeAutomatedPlan(unittest.TestCase):

    def decide_plan(self, steps, constraints):
        planner = Planner()
        planner.initializeTaskGraph(steps)
        planner.updateTaskGraph(constraints)
        return planner

    def assert_plan(self, planner):
        plan = []
        done = False
        while (not done):
            next_move = planner.decideNextMove()
            if (not next_move):
                done = True
            else:
                plan.append(next_move)
        planner.print_list(plan)
        self.assertTrue(self.plan_is_valid(plan))

    def plan_is_valid(self, plan):
        result = True
        copy = list(plan)
        for move in plan:
            if ((move == 'cranfieldStep_006') and ('cranfieldStep_005' in copy)):
                result = False
            elif  ((move == 'cranfieldStep_007') and ('cranfieldStep_006' in copy)):
                result = False
            copy.remove(move)
        return result

    def runTest(self):
        for i in range(10):
            rdf = RdfManager("http://webprotege.stanford.edu/cogrob.tut/", "/home/anglerau/Franka/catkin_ws/src/cog-rob-tut/franka_tut_reasoning/owl/")
            rdf.load("root-ontology.owl")
            """ Retrieve useful information about the task """
            steps = rdf.getTaskSteps("Cranfield_001")
            constraints = rdf.getTaskConstraints("Cranfield_001")
            self.assert_plan(self.decide_plan(steps, constraints))


if __name__ == '__main__':
    rosunit.unitrun('franka_tut_reasoning', 'test_plan_making', MakeAutomatedPlan)
