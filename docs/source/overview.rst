Overview
========

The system presented here allows a human to make simple queries to a robot. If they are not understood the robot asks 
for explanations about the unknown symbol(s). 

The following use cases are considered for now :

 * Action is not understood. It needs to be defined with a sequence of actions already known.
 
 * Target is not understood. It can be defined either as a sub element or as an equivalent of a concept already known.

.. important::

   The system is highly dependent on the quality of the inputs provided to the robot. It is thus important that you make sure 
   whether your microphone is good enough or not.


The main components are:

* ``cogrob_tut_speech_recognition``
* ``cogrob_tut_reasoning``
