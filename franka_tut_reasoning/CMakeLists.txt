cmake_minimum_required(VERSION 2.8.3)
project(franka_tut_reasoning)

find_package(catkin REQUIRED rospy)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/reasoning_node.py
  scripts/record_motion_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
