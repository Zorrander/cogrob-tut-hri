#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *


class OWLClass():
    def __init__(self, category, symbol):
        self.uri_fragment = symbol
        self.uri = make_uri(category, symbol)
        self.category = category
        self.type = RDFTriple(self.uri, "rdf:'type'", make_uri('owl', 'Class'))
        self.subclass_of = []
        self.equivalent_of = []
        self.constraints = []

    def add_constraint(self, values_from):
        self.constraints.append(Restriction(values_from))

    def pl_assert(self):
        '''return list of queries'''
        list_queries = []
        list_queries.append(self.type.generate_assert_query())
        for sub in self.subclass_of:
            list_queries.append(sub.generate_assert_query())
        for sub in self.equivalent_of:
            list_queries.append(sub.generate_assert_query())
        for c in self.constraints:
            list_queries.append(c.generate_assert_query())
        return list_queries

    def export(self):
        if (self.category=="actions"):
            return("export_action("+self.uri+", '/home/ubu2_1404/cob_catkin_ws/result/"+self.uri_fragment+".owl')")
        else:
            return("export_object_class("+self.uri+", '/home/ubu2_1404/cob_catkin_ws/result/"+self.uri_fragment+".owl')")

class RDFTriple():
    def __init__(self, subject, predicate, obj):
        self.subject=subject
        self.predicate=predicate
        self.object=obj

    def generate_assert_query(self):
        return "rdf_assert("+self.subject+", "+self.predicate+", "+self.object+")"

class Restriction():
    def __init__(self, values_from):
        self.on_property = make_uri('knowrob', 'subAction')
        self.values_from = make_uri('actions', values_from)

    def generate_assert_query(self):
        return("owl_restriction_assert(restriction("+self.on_property+",some_values_from("+self.values_from+")), Id)")
