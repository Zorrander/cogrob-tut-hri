.. Human-Robot Interactive Learning documentation master file, created by
   sphinx-quickstart on Tue Mar  6 22:44:00 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Human-Robot Interactive Learning documentation
============================================================

The Human-Robot Interactive Learning project is composed of several ROS packages that can be found on
`GitHub <https://github.com/Zorrander/cogrob-tut-hri>`_. We welcome any suggestion for improvements.

The two main components of the project are :
 * ``cogrob_tut_speech_recognition``, in charge of delivering meaningful tokens
 to the reasoning module from natural language inputs provided by the user.

 * ``cogrob_tut_reasoning``, analyzing the tokens of a spoken request to
 determine if it is able to perform it or not.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   overview
   requirements
   installation
   getting_started
   speech_recognition
   reasoning
   troubleshooting
