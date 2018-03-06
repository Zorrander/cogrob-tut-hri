#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Prolog module.

Are defined here the functions interacting with the prolog predicates

"""
import roslib; roslib.load_manifest('json_prolog')

from json_prolog import json_prolog
from cogrob_tut_reasoning.model import *

from errors import *
from utils import *

from cogrob_tut_msgs.msg import *
from cogrob_tut_msgs.srv import *

class PrologInterface():
    """Class for interactions with prolog methods."""

    def __init__(self):
        self.prolog = json_prolog.Prolog()

    def has_property(self, individual, propertie, value):
        ''' Check the existence of the property for the individual given'''
        try:
            result = False
            query = "owl_has("+ individual +","+ propertie +"," + value + ")"
            print query
            result = self.prolog.once(query)
        except:
            pass
        else:
            print result
            if (result == []):
                pass
            if (result == {}):
                result = True
            return result

    def has_individual(self, subClass, category):
        ''' Check the existence of the individual for the class given, with some optional properties
        '''
        result = []
        uriClass = make_uri(category, subClass)
        query = "owl_individual_of(Ind, " + uriClass + ")"

        result_query = self.prolog.once(query)
        if not result_query:
            pass
        else:
            for solution in self.prolog.query(query).solutions():
                result.append(solution['Ind'])
        return result

    def has_sub_class(self, subClass, category):
        ''' Check the existence of the sub class for the class given.'''
        uriClass = make_uri(category, 'Independent_entity')
        uriSubClass = make_uri(category, subClass)
        query = "rdfs_subclass_of(" + uriSubClass + ", " + uriClass + ")"
        result_query = self.prolog.once(query)
        if result_query == {}:
            return True
        else :
            return False

    def has_equivalent_class(self, subClass):
        ''' Check the existence of an equivalent class. Useful to spot synonyms or similar concepts'''
        uriClass = make_uri('knowrob', subClass)
        predicate = make_uri('owl', 'equivalentClass')
        query = self.prolog.once("owl_has(EquivalentClass,"+ predicate +"," + uriClass + ")")
        if not query:
            return ""
        else:
            return get_uri_fragment(query['EquivalentClass'])

    def get_all_potential_targets(self):
        result = []
        for solution in self.prolog.query("rdfs_subclass_of(Subclass," + make_uri('knowrob', "SpatialThing") + ")").solutions():
            result.append(get_uri_fragment(solution['Subclass']))
        return result

    def get_all_potential_actions(self):
        result = []
        for solution in self.prolog.query("rdfs_subclass_of(Subclass," + make_uri('knowrob', "Event") + ")").solutions():
            result.append(get_uri_fragment(solution['Subclass']))
        return result

    def get_all_properties_entity(self, entity):
        result = []
        for solution in self.prolog.query("owl_has(" + entity + ", Predicate, Object)").solutions():
            result.append(get_uri_fragment(solution['Predicate']))
            result.append(get_uri_fragment(solution['Object']))
        return result

    def check_actions(self, action):
        actions = self.get_all_potential_actions()
        if (not (action in actions)):
            raise FailedToUnderstandAction(action)

    def check_targets(self, target):
        targets = self.get_all_potential_targets()
        if (not (target in targets)):
            raise FailedToUnderstandTarget(target)

    ##################################################################################################

    def pl_assert(self, symbol):
        for query in (symbol.pl_assert()):
            print query
            result = self.prolog.once(query)
            if result:
                self.prolog.once(RDFTriple(symbol.uri, make_uri('rdfs', 'subClassOf'), result['Id']).generate_assert_query())
                self.prolog.once(RDFTriple(symbol.uri, make_uri('rdfs', 'subClassOf'), make_uri('actions', 'Independent_entity')).generate_assert_query())


    def ground_new_symbol(self, triples, server_type, symbol):
        new_symbol = OWLClass(server_type, symbol)
        ''' Teach a new symbol to the robot'''
        if (server_type=="actions"):
            for triple in triples:
                new_symbol.add_constraint(triple[0])
        else:
            for triple in triples:
                new_symbol.subclass_of.append(RDFTriple(new_symbol.uri, make_uri('rdfs', 'subClassOf'), make_uri('targets', 'Independent_entity')))
                if (triple[0] == "Like"): # Grounding equivalent
                    equivalent_class_property = make_uri('owl', 'equivalentClass')
                    if self.has_sub_class(triple[1], server_type):
                        new_symbol.equivalent_of.append(RDFTriple(new_symbol.uri, equivalent_class_property , make_uri(server_type, triple[1])))
                    else:
                        new_symbol.equivalent_of.append(RDFTriple(new_symbol.uri, equivalent_class_property, make_uri('knowrob', triple[1])))
                else:
                    subclass_property = make_uri('rdfs', 'subClassOf')
                    if self.has_sub_class(triple[1], server_type):
                        new_symbol.subclass_of.append(RDFTriple(new_symbol.uri, subclass_property, make_uri(server_type, triple[1])))
                    else:
                        new_symbol.subclass_of.append(RDFTriple(new_symbol.uri, subclass_property, make_uri('knowrob', triple[1])))

        self.pl_assert(new_symbol)
        self.prolog.once(new_symbol.export())
        print "export done"

    def ground_equivalent_property(self, symbol, symbol_already_known, category):
        subject = make_uri(category, symbol)
        predicate = make_uri('owl', 'equivalentClass')
        obj = make_uri(category, symbol_already_known)
        grounding = self.pl_assert(subject, predicate, obj)
        return grounding


    def retrieve_actions(self, request):
        actions = self.get_all_potential_actions()
        result = retrieveActionsResponse()
        for action in actions:
            item = Action()
            item.name = action
            result.kb_actions.append(item)
        return result

    def retrieve_targets(self, request):
        targets = self.get_all_potential_targets()
        result = retrieveTargetsResponse()
        for target in targets:
            item = Target()
            item.name = target
            result.kb_targets.append(item)
        return result
