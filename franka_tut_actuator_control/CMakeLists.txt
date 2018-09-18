cmake_minimum_required(VERSION 2.8.3)
project(franka_tut_actuator_control)

find_package(catkin REQUIRED rospy)

catkin_python_setup()

catkin_package()


install(PROGRAMS
 scripts/actuator_node.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
