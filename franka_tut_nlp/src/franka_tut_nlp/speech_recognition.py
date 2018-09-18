"""

Conversation module.

"""

import rospy
import actionlib
import nltk
from nltk.corpus import stopwords

from std_msgs.msg import String, Empty
from franka_tut_msgs.msg import *
from franka_tut_nlp.state_machine import StateMachine


class Conversation(object):
    pub_relate_skill = rospy.Publisher("grounding_skill_link", Instructions, queue_size=10)
    processed_input_publisher = rospy.Publisher('processed_inputs', Instruction, queue_size=10)
    publisher_ss = rospy.Publisher('speech_synthesizer', String, queue_size=10)
    pub_grounding_triple = rospy.Publisher("grounding_triple", Instruction, queue_size=10)
    pub_end_grounding = rospy.Publisher("end_grounding", Empty, queue_size=10)

    def __init__(self):
        '''Initialize state machine'''
        self.m = StateMachine()
        self.m.add_state("listening", self.listening_transitions)
        self.m.add_state("grounding_action", self.grounding_action_transitions)
        self.m.add_state("grounding_target", self.grounding_target_transitions)
        self.m.add_state("performing", self.busy_transitions)
        self.m.add_state("error_state", None, end_state=1)
        self.m.set_state("listening")

    def sr_processing(self, message):
        tokens = self.preprocess_txt(message.data)
        self.m.run(tokens)

    def grounding_active(self):
        print "Task active !"

    def grounding_feedback(self, feedback):
        print "Task feedback"

    def grounding_done(self, state, result):
        print "Task completed with success !"

    def symbol_unknown(self, msg):
        self.m.set_state("listening")

    def preprocess_txt(self, txt):
        list_words = ['oh', 'ah', 'okay', 'ok', 'well', 'please', 'first', 'then', 'finally', 'listening', 'understood', 'got', 'it', 'explain', 'cocaine', 'another', 'way', 'room']
        banned_words = stopwords.words('english') + list_words
        bag_words = [word for word in nltk.word_tokenize(txt.lower()) if word.isalpha()]
        tokens = [t.title() for t in bag_words if t not in banned_words]
        return tokens

    def listening_transitions(self, tokens):
        '''
        process tokens in the listening mode
        '''
        if tokens[0] == "Teach":
            new_state = self.listening_to_teaching(tokens)
        else:
            if (len(tokens) == 2):  # received valid command
                new_state = "performing"
                Conversation.processed_input_publisher.publish(Instruction(tokens[0], tokens[1]))
            else:  # received illicit command
                new_state = "listening"
                symbols = tokens
        print ("Going to state --> %s" % new_state.upper())
        return new_state

    def listening_to_teaching(self, tokens):
        '''
        activate either the action grounding or target grounding mode
        '''
        print tokens
        if (len(tokens) == 2):
            new_state = "grounding_target"
            goal = AddInfoConversationGoal(symbol=tokens[1], category="targets")
            client = actionlib.SimpleActionClient('grounding_action', AddInfoConversationAction)
            client.wait_for_server()
            client.send_goal(goal, self.grounding_done, self.grounding_active, self.grounding_feedbackb)
        elif (tokens[1] == "Action"):
            new_state = "grounding_action"
            goal = AddInfoConversationGoal(symbol=tokens[2], category="actions")
            client = actionlib.SimpleActionClient('grounding_action', AddInfoConversationAction)
            client.wait_for_server()
            client.send_goal(goal, self.grounding_done, self.grounding_active, self.grounding_feedback)
        else:
            new_state = "listening"
        return new_state

    def grounding_action_transitions(self, tokens):
        '''
        process tokens in the listening mode
        '''
        print tokens
        if (tokens[0] == "Done"): # End of grounding process
            new_state = "listening"
            Conversation.pub_end_grounding.publish()
        elif (len(tokens) == 2):  # building skill with a sequence of known utterances
            new_state = "grounding_action"
            msg = create_instruction_msg(Instruction(tokens[0], tokens[1]))
            Conversation.pub_grounding_triple.publish(msg)
        elif (len(tokens) == 5):  # relating to an existing skill
            new_state = "listening"
            instructions = [Instruction(tokens[0], tokens[1]), Instruction(tokens[3], tokens[4])]
            Conversation.pub_relate_skill.publish(Instructions(instructions))
        else:
            new_state = "listening"
        print ("Going to state --> %s" % new_state.upper())
        return new_state

    def grounding_target_transitions(self, tokens):
        '''
        process tokens in the listening mode
        '''
        if (tokens[0] == "Done"): # End of grounding process
            new_state = "listening"
            Conversation.pub_end_grounding.publish()
        elif (len(tokens) == 1): # Defining a sub classs
            new_state = "grounding_target"
            Conversation.pub_grounding_triple.publish(Instruction("", tokens[0]))
        elif (len(tokens) <= 3): # Defining an equivalent
            if (tokens[0] == "Like"):
                new_state = "grounding_target"
                Conversation.pub_grounding_triple.publish(Instruction(tokens[0], tokens[1]))
            elif (tokens[1] == "Like"):
                new_state = "grounding_target"
                Conversation.pub_grounding_triple.publish(Instruction(tokens[1], tokens[2]))
            else:
                new_state = "grounding_target"
                Conversation.pub_grounding_triple.publish(Instruction("", tokens[1]))
        else:
            new_state = "listening"
        print ("Going to state --> %s" % new_state.upper())
        return new_state

    def busy_transitions(self, tokens):
        '''
        process tokens in the performing mode
        '''
        if (tokens[0] == "Stop"):
            new_state = "listening"
            print ("Going to state --> %s" % new_state.upper())
        else:
            new_state = "performing"
            print "Cannot take other input while performing an action"
        return new_state
