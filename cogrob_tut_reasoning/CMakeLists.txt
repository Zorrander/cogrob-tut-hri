cmake_minimum_required(VERSION 2.8.3)
project(cogrob_tut_reasoning)

find_package(catkin REQUIRED
  COMPONENTS
  rospy
  cogrob_tut_msgs
  knowrob_common
  json_prolog
)

catkin_python_setup()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cogrob_tut_reasoning
   CATKIN_DEPENDS rospy cogrob_tut_msgs knowrob_common json_prolog
#  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/reasoning_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
