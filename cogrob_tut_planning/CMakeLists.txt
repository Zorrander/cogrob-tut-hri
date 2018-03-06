cmake_minimum_required(VERSION 2.8.3)
project(cogrob_tut_planning)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cogrob_tut_planning
   CATKIN_DEPENDS rospy
#  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/planning_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
