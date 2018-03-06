#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Conversation module."""

import rospy

from .state_machine import StateMachine
from .rospy_helper import *

import nltk
from nltk.corpus import stopwords


def preprocess_txt(txt):
    list_words = ['oh', 'ah', 'okay', 'ok', 'well', 'please', 'first', 'then', 'finally', 'listening', 'understood', 'got', 'it', 'explain', 'cocaine', 'another', 'way', 'room']
    banned_words = stopwords.words('english') + list_words
    bag_words = [word for word in nltk.word_tokenize(txt.lower()) if word.isalpha()]
    tokens = [t.title() for t in bag_words if t not in banned_words]
    return tokens

class Conversation(object):

    def __init__(self, pub_reasoner, pub_ss, pub_grounding_triple, pub_end_grounding):
        """ Initializer """
        self.pub_reasoner = pub_reasoner
        self.pub_ss = pub_ss
        self.pub_grounding_triple = pub_grounding_triple
        self.pub_end_grounding = pub_end_grounding

        '''Initialize state machine'''
        self.m = StateMachine()
        self.m.add_state("listening", self.start_transitions)
        self.m.add_state("grounding_action", self.grounding_action_transitions)
        self.m.add_state("grounding_target", self.grounding_target_transitions)
        self.m.add_state("performing", self.busy_transitions)
        self.m.add_state("error_state", None, end_state=1)
        self.m.set_state("listening")
        ''' ====================== '''

    def sr_processing(self, message):
        tokens = preprocess_txt(message.data)
        print tokens
        self.m.run(tokens)

    def doneCb(self, state, result):
        print "Task completed with success !"

    def activeCb(self):
        print "Task active !"

    def feedbackCb(self, feedback):
        print "Task feedback"

    def start_transitions(self, tokens):
        if tokens[0] == "Teach":
            if (len(tokens) == 2):
                new_state = "grounding_target"
                goal = AddInfoConversationGoal(symbol=tokens[1], category="targets")
                create_grounding_client(goal, self.doneCb, self.activeCb, self.feedbackCb)
            elif (tokens[1] == "Action"):
                new_state = "grounding_action"
                goal = AddInfoConversationGoal(symbol=tokens[2], category="actions")
                create_grounding_client(goal, self.doneCb, self.activeCb, self.feedbackCb)
            else:
                new_state = "listening"
                symbols = tokens
        else:
            if (len(tokens) == 2):
                new_state = "performing"
                symbols = tokens
                msg = create_instruction_msg(symbols[0], symbols[1])
                self.pub_reasoner.publish(msg)
            else:
                new_state = "listening"
                symbols = tokens
        print ("Going to state --> %s" % new_state.upper())
        return new_state

    def grounding_action_transitions(self, tokens):
        first_token=tokens[0]
        if (tokens[0] == "Done"): # End of grounding process
            new_state = "listening"
            self.pub_end_grounding.publish()
        elif (len(tokens) == 2): # Defining a sub Task
            new_state = "grounding_action"
            msg = create_instruction_msg(tokens[0], tokens[1])
            self.pub_grounding_triple.publish(msg)
        else:
            new_state = "listening"
        print ("Going to state --> %s" % new_state.upper())
        return new_state

    def grounding_target_transitions(self, tokens):
        if (tokens[0] == "Done"): # End of grounding process
            new_state = "listening"
            self.pub_end_grounding.publish()
        elif (len(tokens) == 1): # Defining a sub classs
            new_state = "grounding_target"
            msg = create_instruction_msg("", tokens[0])
            self.pub_grounding_triple.publish(msg)
        elif (len(tokens) <= 3): # Defining an equivalent
            if (tokens[0] == "Like"):
                new_state = "grounding_target"
                msg = create_instruction_msg(tokens[0], tokens[1])
                self.pub_grounding_triple.publish(msg)
            elif (tokens[1] == "Like"):
                new_state = "grounding_target"
                msg = create_instruction_msg(tokens[1], tokens[2])
                self.pub_grounding_triple.publish(msg)
            else:
                new_state = "grounding_target"
                msg = create_instruction_msg("", tokens[1])
                self.pub_grounding_triple.publish(msg)
        else:
            new_state = "listening"
        print ("Going to state --> %s" % new_state.upper())
        return new_state

    def busy_transitions(self, tokens):
        if (tokens[0] == "Stop"):
            new_state = "listening"
            print ("Going to state --> %s" % new_state.upper())
        else:
            new_state = "performing"
            print "Cannot take other input while performing an action"
        return new_state

    def symbol_unknown(self, symbol):
        self.m.set_state("listening")
        print ("Going to state --> %s" % self.m.currentState.upper())
