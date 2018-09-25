"""Knowledge base module.

Are defined here the different requests to the knowledge.
Reasoning about the RDF results is also included.

"""


import rospy
import requests
from franka_tut_msgs.msg import *
from franka_tut_msgs.srv import *
from franka_tut_reasoning.errors import *


ENDPOINT = "http://127.0.0.1:5000/endpoint/"


class RdfManager(object):
    """docstring for RdfManager."""

    def send_query(self, query):
        """Retrieve matches to a given query.

        Reach the query url and use json to embbed the query itself.

        Parameters
        ----------
        query : sparql
                The query

        Returns
        -------
        List[str]
            The list of matches.
        """
        payload = {'query':query}
        r = requests.get(url=ENDPOINT, json=payload)  # sending get request and saving the response as response object
        data = r.json()  # extracting data in json format +
        return [x for x in data['result']]

    def get_skills(self):
        """Retrieve skills available.

        Returns
        -------
        List[str]
            The list of skills.
        """
        r = requests.get(url=ENDPOINT+"skills")
        data = r.json()
        return [x for x in data['result']]

    def get_task_steps(self, task_name):
        """Retrieve skills available.

        Parameters
        ----------
        task_name : str
                the name of a skill

        Returns
        -------
        List[str]
            The list of steps for this skill.
        """
        r = requests.get(url=ENDPOINT+"steps/"+task_name)
        data = r.json()
        rospy.logdebug("get_task_steps request gave : {}".format(data))
        return [x for x in data['result']]

    def get_step_info(self, command_name):
        """Retrieve information available for a given skill.

        Retrieve the skill activated by the command sent as argument, list its steps,
        and for each one of them gets all information available.

        Parameters
        ----------
        command_name : str
                An utterance.

        Returns
        -------
        List[(x,y)]
            The list of (predicate, object).
        """
        r = requests.get(url=ENDPOINT+"step/"+command_name)
        data = r.json()
        rospy.logdebug("get_step_info request gave : {}".format(data))
        return data

    def find_utterances(self, action):
        """Retrieve utterances available in the knowledge base that match the command.

        Parameters
        ----------
        action : str
                An action extracted from an user request.

        Returns
        -------
        List[str]
            The list of matching utterances.
        """
        r = requests.get(url=ENDPOINT+"utterances/"+action)
        data = r.json()
        rospy.logdebug("filter_utterances request gave : {}".format(data))
        return data["result"]

    def get_following_slot(self, slot):
        """Retrieve the slot linked to the argument.

        Parameters
        ----------
        slot : str
                An action extracted from an user request.

        Returns
        -------
        List[str]
            The potential targets linked to this action.
        """
        r = requests.get(url=ENDPOINT+"next_slot/"+slot)
        data = r.json()
        rospy.logdebug("get_following_slot request gave : {}".format(data))
        return data

    def get_task_from_step(self, step):
        """Retrieve the tasks that compose a step from a skill.

        Parameters
        ----------
        step : str
                Name of the step.

        Returns
        -------
        List[str]
            The tasks that are involved in this step.
        """
        r = requests.get(url=ENDPOINT+"task/"+step)
        data = r.json()
        rospy.logdebug("get_task_from_step request gave : {}".format(data))
        return data["result"]

    def ground_subclass(self, new_class, mother_class):
        """Activate grounding process for a new subclass.

        Parameters
        ----------
        new_class : str
                Name of the new class to add.

        mother_class : str
                Name of the mother class
        """
        r = requests.get(url=ENDPOINT+"ground/subclass/"+new_class+"/"+mother_class)
        data = r.json()
        return data['result']

    def relate_new_skill(self, new_skill, related_skill):
        """Add new utterance as trigger for a given skill.

        Parameters
        ----------
        new_skill : (action, target)
                Signature of the new utterance.
        related_skill : (action, target)
                Signature of the reference.
        """
        r = requests.get(url=ENDPOINT+"relate/skill/"+new_skill.action+"/"+new_skill.target+"/"+related_skill.action+"/"+related_skill.target)

    '''
    def get_first_task_from_step(self, step):
        """Retrieve the tasks that compose a step from a skill.

        Parameters
        ----------
        step : str
                Name of the step.

        Returns
        -------
        List[str]
            The tasks that are involved in this step.
        """
        r = requests.get(url=ENDPOINT+"first/task/"+step)
        data = r.json()
        rospy.logdebug("get_first_task_from_step request gave : {}".format(data['result']))
        return data['result'][0]

    def rec_extract_actions_from_task(self, task):
        task = self.get_task_from_step(task)
        if task[0]: #hasAction
            return [task[0]]
        else:
            does_first = [task[1]]
            does_then = task[2]
            result = does_first + self.rec_extract_actions_from_task(does_then)
            rospy.logdebug("rec_extract_actions_from_task returns : {}".format(result))
            return result

    def extract_actions_from_step(self, step):
        first_task = self.get_first_task_from_step(step)
        res = self.rec_extract_actions_from_task(first_task)
        return res

    def extract_actions_from_task(self, task):
        r = requests.get(url=ENDPOINT+"actions/"+task)
        data = r.json()
        rospy.logdebug("extract_actions_from_task request gave : {}".format(data))
        return data

    def get_action_preconditions(self, action):
        r = requests.get(url=ENDPOINT+"preconditions/"+action)
        data = r.json()
        rospy.logdebug("get_action_preconditions request gave : {}".format(data))
        return data['result']

    def retrieve_location(self, entity):
        r = requests.get(url=ENDPOINT+"location/"+entity)
        data = r.json()
        rospy.logdebug("retrieve_location request gave : {}".format(data))
        return data['result']

    def retrieve_description(self, entity):
        r = requests.get(url=ENDPOINT+"description/"+entity)
        data = r.json()
        rospy.logdebug("retrieve_width request gave : {}".format(data))
        if data['result'] :
            return data['result'][0]
        else:
            return ""

    def retrieve_width(self, entity):
        r = requests.get(url=ENDPOINT+"width/"+entity)
        data = r.json()
        rospy.logdebug("retrieve_width request gave : {}".format(data))
        return data['result']

    def check_attribute(self, attribute):
        r = requests.get(url=ENDPOINT+"attributes/"+attribute)
        data = r.json()
        rospy.logdebug("check_attribute({}) -> {}".format(attribute, data))
        return data['result']
    '''

class TaskSemanticManager(RdfManager):
    """docstring for TaskSemanticManager."""
    def __init__(self):
        super(TaskSemanticManager, self).__init__()
        self.available_tasks = self.get_skills()
        '''
        self.current_context = []
        self.max_width_gripper = 80.
        self.dictionary_preconditions = {
                'TargetReachable_Constraint': self.is_target_reachable,
                'TargetGraspable_Constraint': self.is_target_graspable,
                'GripperOperational_Constraint': self.is_gripper_operational,
                'ValidRotationAngle_Constraint': self.is_rotation_angle_valid
        }'''

    def check_syntax(self, action, target):
        ''' Match the request with the skills having a compatible syntax '''
        potential_utterances = self.find_utterances(action)  # First retrieve the commands corresponding to the first symbol
        if potential_utterances:
            validater = self.compare_command(target)
            valid_utterances = list(filter(validater, potential_utterances))
            if not valid_utterances:
                raise RequestSyntax_Error
            return valid_utterances
        else:
            raise RequestSyntax_Error(action)

    def compare_command(self, target):
        def matching_uterrance(utterance):
            ''' Compares the Nth attribute with registered Nth slot of the utterance '''
            rospy.logdebug("Analyzing utterance {}".format(utterance))
            next_slot = self.get_following_slot(utterance)
            depth=0
            while not next_slot["name"] == "End_Command":
                if (next_slot["value"]):
                    if not (target[depth] and next_slot["value"] == target[depth]):
                        return False
                next_slot = self.get_following_slot(next_slot["name"])
                depth+=1
            return True

    def check_semantic(self, action, target, set_skills):
        ''' Solves the ambiguities if necessary to retrieve only one skill to trigger in answer of a request '''
        if (len(set_skills)==1):
            rospy.logdebug("No ambiguity about {}".format(set_skills[0]))
            map(self.matching_attribute, target) if isinstance(target, list) else self.matching_attribute(target)
            return set_skills[0]
        else:
            raise RequestSemantic_Error

    def ground_new_symbol(self, items, server_type, symbol):
        if (server_type=="actions"):
            pass
        else:
            for action, target in items:
                self.ground_subclass(target, symbol)
    """
    def matching_attribute(self, attribute):
        found = self.check_attribute(attribute)
        if not found:
            raise TargetUnknown_Error(attribute)

    def comply_to_preconditions(self, node):
        actions = node[1]
        rospy.logdebug("comply_to_preconditions(node : {})".format(actions))
        preconditions = list(map(self.get_action_preconditions, actions))
        actions.append({})
        rospy.logdebug("Found the following preconditions {} for node {}".format(preconditions, actions))
        for action_conditions in preconditions:
            for fun in list(map(self.dictionary_preconditions.get, action_conditions)):
                fun(actions)

    def is_target_reachable(self, node):
        ''' Checks if the target is known and maps it to its location'''
        rospy.logdebug("is_target_reachable({}, {})".format(node, self.current_context))
        if 'target' in node:
            pass  # TODO
        else:
            target = self.current_context # TODO
            location = self.retrieve_location(target)
            if not location:
                raise LocationUnknown_Error(target)
            node[-1].update({'location_target': location})

    def is_target_graspable(self, node):
        ''' Checks if the width of the object is less than max width of the gripper'''
        rospy.logdebug("is_target_graspable({}, {})".format(node, self.current_context))
        if 'target' in node:
            pass  # TODO
        else:
            target = self.current_context # TODO
            width = float(self.retrieve_width(target)[0])
            if not width:
                raise WidthUnknown_Error(target)
            elif (width >= self.max_width_gripper):
                raise Grasping_Error(target)
            node[-1].update({'width_target': str(width)})

    def is_gripper_operational(self, node):
        rospy.logdebug("is_gripper_operational({}, {})".format(node, self.current_context))
        # Working status
        # Available state

    def is_rotation_angle_valid(self, node):
        rospy.logdebug("is_rotation_angle_valid({}, {})".format(node, self.current_context))
        node[-1].update({'angle_goal': str(20)})

    def procedural_attachment(self, steps, context):
        self.current_context = context
        map(self.comply_to_preconditions, steps)

    def has_constraint(self, step):
        if 'isDoneAfter' in step[0]:
            return (step[0]['isDoneAfter'], step[1])

    def retrieve_actions(self, skill):
        rospy.logdebug("Received handle_plan_query about :{}".format(skill))
        steps = self.get_step_info(skill)
        f_steps = []
        for i in range(len(steps)):
            f_steps.append((steps[steps.keys()[i]], steps.keys()[i]))
        constraints = list(map(self.has_constraint, f_steps))
        tasks = [steps[steps.keys()[i]]["consistsIn"] for i in range(len(steps))]
        step_result = []
        for i in range(len(tasks)):
            motions = self.rec_extract_actions_from_task(tasks[i])
            step_result.append((steps.keys()[i], motions))
        return (step_result, constraints)
    """
