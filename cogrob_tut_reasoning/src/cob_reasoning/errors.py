#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Errors module.

Are defined here the errors for the cob reasoning module

"""

class Error(Exception):
    """Base class for exceptions in this module."""
    def __init__(self):
        pass

class FailedToUnderstandAction(Error):
    """Exception raised when the action is not understood"""

    def __init__(self, symbol):
        self.message = "Please explain " + symbol + " in another way"

class FailedToUnderstandTarget(Error):
    """Exception raised when the request is not understood"""

    def __init__(self, symbol):
        self.message = "Please explain " + symbol + " in another way"
