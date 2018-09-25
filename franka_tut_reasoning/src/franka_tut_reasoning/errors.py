"""Errors module.

Are defined here the errors for the reasoning module

"""

class Error(Exception):
    """Base class for exceptions in this module."""
    def __init__(self):
        pass

class RequestSyntax_Error(Error):
    """ Exception raised when the request cannot be matched to any utterance """

    def __init__(self, symbol):
        self.value = symbol
        self.message = "I am sorry, " + symbol + " does not match with any result"

class RequestSemantic_Error(Error):
    """ Exception raised when the request can be matched to several utterances """

    def __init__(self):
        self.message = "I am sorry, the request matches with more than one result"

class TargetUnknown_Error(Error):
    """ Exception raised when one of the attribute of the request is not known """

    def __init__(self, symbol):
        self.value = symbol
        self.message = "Please explain " + symbol + " in another way"

class LocationUnknown_Error(Error):
    """ Exception raised when the action requires a motion to un undefined location """

    def __init__(self, symbol):
        self.message = "Please tell me where " + symbol + " is"

class WidthUnknown_Error(Error):
    """ Exception raised when the width of the object to grasp is not know is not known """

    def __init__(self, symbol):
        self.value = symbol
        self.message = "I am not sure if I can grasp " + symbol + " because I don't know the width"

class Grasping_Error(Error):
    """ Exception raised when the width of the object to grasp is superior to the maximum width of the gripper """

    def __init__(self, symbol):
        self.value = symbol
        self.message = "I cannot grasp" + symbol + " as it is too wide for my current gripper"
