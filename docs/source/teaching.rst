.. _teaching:


Teaching a new skill
====================

First of all some basic information about the skill need to be stored to enable reasoning about this knowledge later on.
On the web app, a new skill can be taught in three stages.

Name of the skill
^^^^^^^^^^^^^^^^^

To differentiate them from each other in the knowledge base and to be displayed in the GUI a skill needs a name.
Often a verb indicating the purpose of the skill is used.

How to activate the skill
^^^^^^^^^^^^^^^^^^^^^^^^^

Our interaction module is based on <action>, <target> pairs and thus a skill can be requested by one or several specific pairs
that need to be decided here. Later on if another pair is used, it can be linked to this skill using the first utterances that have been
defined.


Subdivide the skill into steps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To enable collaboration between a human and the robot the robot needs to have a global understanding of the task and
how to share it with another agent. To allow this, the user can split the skill into several logical steps. Each step can be composed
of one or more tasks.
