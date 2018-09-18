#! /usr/bin/env python

import abc
import networkx as nx
import rospy
import matplotlib.pyplot as plt


MAP_STEP_ARG={
        'CartesianMotion': 'location_target',
        'GripperMotion': 'width_target',
        'JointAction': 'angle_goal'
}

class Planner(object):
    """docstring for Planner."""
    __metaclass__  = abc.ABCMeta

    def __init__(self):
        super(Planner, self).__init__()
        self.graph = nx.DiGraph()

    def add_node(self, name, x, agent="Human"):
        """adds a new step"""
        params = x.pop(-1)
        #moves = [move.split('_')[0] for move in x]
        self.graph.add_node(name, value=x, attributes=params, generator=self.generator_move(x, params), executor=agent, done=False)

    def attribute_node_to_human(self, node):
        nx.set_node_attributes(self.graph, {node : {'executor' : 'Human'}})

    def attribute_node_to_robot(self, node):
        nx.set_node_attributes(self.graph, {node : {'executor' : 'Robot'}})

    def complete_step(self, node):
        nx.set_node_attributes(self.graph, {node : {'done' : 'True'}})

    def generator_move(self, moves, params):
        for action in moves:
            print "Generating move for {}".format(action)
            if not action.startswith("NullAction"):
                #yield (action, params[MAP_STEP_ARG.get(action)])
                yield(action)
            else:
                yield(action)

    def print_graph(self):
        print list(self.graph.nodes(data=True))

    def draw_graph(self):
        print "drawing"
        pos=nx.spring_layout(self.graph)
        human_nodes =  [n for n, d in self.graph.nodes.data() if d['executor']=="Human"]
        robot_nodes =  [n for n, d in self.graph.nodes.data() if d['executor']=="Robot"]
        nx.draw_networkx_nodes(self.graph, pos, nodelist=human_nodes, node_color='r', node_size=500, alpha=0.8, label="Handled by humans")
        nx.draw_networkx_nodes(self.graph, pos, nodelist=robot_nodes, node_color='b', node_size=500, alpha=0.8, label="Handled by the robot")
        nx.draw_networkx_edges(self.graph, pos, edgelist=self.graph.edges(), width=8, alpha=0.5, edge_color='g')

        labels = {n: n for n, d in self.graph.nodes.data()}
        nx.draw_networkx_labels(self.graph, pos, labels, font_size=16)
        plt.show()

    @abc.abstractmethod
    def initialize(self, steps, constraints):
        """Create groups of steps depending on the strategy"""

    @abc.abstractmethod
    def decide_next_move(self):
        """Decision making strategy"""

class AutomatedPlanner(Planner):
    """docstring for AutomatedPlanner."""
    def __init__(self):
        super(AutomatedPlanner, self).__init__()

    def initialize(self, steps, constraints):
        rospy.logdebug("initialize graph ({})".format(steps))
        s = []
        for step, tasks in steps:
            self.add_node(step, tasks)
            s.append(step)
        for c in constraints:
            if c:
                self.graph.add_edge(*c)
        self.print_graph()
        for n in s:
            for e in self.graph.edges():
                if e[1] == n:
                    s.pop(s.index(n))
        self.current_step = s[0]
        self.draw_graph()
        self.print_graph()

    def decide_next_move(self):
        """Decides which action to take next"""
        current_node = self.graph.nodes.data()[self.current_step]
        if not current_node['done']:
            try:
                return next(current_node['generator'])
            except StopIteration:
                self.current_step = self.find_next_step()
                if self.current_step:
                    current_node = self.graph.nodes.data()[self.current_step]
                    return next(current_node['generator'])

    def find_next_step(self):
        nodes = [n[0] for n in self.graph.nodes.data() if not n[1]['done']]
        print nodes
        followers = []
        for e in self.graph.edges():
            if e[1] in nodes:
                nodes.pop(nodes.index(e[1]))
                if e[0] == self.current_step:
                    followers.append(e)
        if nodes:
            return nodes[0]
        else:
            if followers:
                return followers[0][1]

class InteractivePlanner(Planner):
    """docstring for InteractivePlanner."""
    def __init__(self):
        super(InteractivePlanner, self).__init__()

    def initialize(self, steps, constraints):
        rospy.logdebug("initialize graph ({})".format(steps))
        s = []
        working_agent = 'Human'
        for step, tasks in steps:
            self.add_node(step, tasks)
            s.append(step)
        for c in constraints:
            if c:
                print ("CONSTRAINTS : {}".format(c))
                if working_agent == 'Robot':
                    self.attribute_node_to_human(c[1])
                    working_agent = 'Human'
                else:
                    self.attribute_node_to_robot(c[1])
                    working_agent = 'Robot'
                self.graph.add_edge(*c)
        self.current_step = self.find_first_step()
        self.draw_graph()
        self.print_graph()

    def find_first_step(self):
        """Identifies the first action required by the robot"""
        poll = []
        possible_first = True
        for edge1 in self.graph.edges():
            for edge2 in self.graph.edges():
                if edge1[0] == edge2[1]:
                    possible_first = False
            if possible_first:
                poll.append(edge1[0])
        print "Possible first : {}".format(poll)
        return poll[0]

    def decide_next_move(self):
        """Decides which action to take next"""
        try:
            if self.current_step:
                current_node = self.graph.nodes.data()[self.current_step]
                print "CURRENT NODE : {}".format(current_node)
                if (current_node['executor'] == 'Human'):
                    return (False, True)
                else:
                    return (next(current_node['generator']), False)
            else:
                return (False, False)
        except StopIteration:
            current_node['done'] = True
            self.find_next_step()
            if self.current_step:
                current_node = self.graph.nodes.data()[self.current_step]
                if (current_node['executor'] == 'Human'):
                    return (False, True)
                else:
                    return (next(current_node['generator']), False)
            else:
                return (False, False)

    def find_next_step(self):
        has_next = False
        self.complete_step(self.current_step)
        for e in self.graph.edges():
            if e[0] == self.current_step:
                print "EDGE : {} == > {}".format(e[0], e[1])
                has_next = e[1]
        self.current_step = has_next

class CollaborativePlanner(Planner):
    """docstring for CollaborativePlanner."""
    def __init__(self, ):
        super(CollaborativePlanner, self).__init__()

    def decide_next_move(self):
        """Decides which action to take next"""
        pass
