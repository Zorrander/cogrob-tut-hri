.. _start:

Getting started
===============

Three steps are necessary in order to start using the robot together with the reasoning system.

.. note::

   These are valid for use with the Panda arm from Franka Emika but would need to be adapted if another robot is used.


Starting the robot and its interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: shell

    $  roslaunch franka_tut bringup_franka.launch

Starting the knowledge base server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


.. code-block:: shell

    $  cd path/to/your/flask/app/
    $  . venv/bin/activate
    $  export FLASK_APP=name-of-your-flask-app
    $  export FLASK_ENV=development
    $  flask run


Starting the reasoning system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


.. code-block:: shell

    $  roslaunch franka_tut franka_tut.launch mode:=<automated|interactive>
