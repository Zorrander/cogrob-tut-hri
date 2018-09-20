Installation instructions
=========================

This chapter describes how to install the knowledge base server and the reasoning system to start teaching and
performing tasks in collaboration with your robot.

.. note::

   Because for this project we use the Panda arm from Franka Emika we use ubuntu 16.04 together with ROS kinetic.
   The following instructions might therefore only work in this environment.



Installing the knowledge base server
------------------------------------

The knowledge base server has been developed with Flask. To get it properly running on your local machine please follow the
instruction from the `Flask documentation <http://flask.pocoo.org/docs/1.0/installation/>`__.

Once Flask is installed clone the ``robot-semweb`` repository from `GitHub <https://github.com/Zorrander/robot-semweb>`__ inside
your environment::

    git clone https://github.com/Zorrander/robot-semweb


Installing the reasoning system
-------------------------------

After setting up ROS Kinetic, create a Catkin workspace in a directory of your choice:

.. code-block:: shell

    cd /path/to/desired/folder
    mkdir -p catkin_ws/src
    cd catkin_ws
    source /opt/ros/kinetic/setup.sh
    catkin_init_workspace src

Then clone the ``cogrob-tut-hri`` repository from `GitHub <https://github.com/Zorrander/cogrob-tut-hri>`__::

    git clone https://github.com/Zorrander/cogrob-tut-hri

Build the packages:

.. code-block:: shell

    catkin_make
    source devel/setup.sh
