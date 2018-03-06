cmake_minimum_required(VERSION 2.8.3)
project(cogrob_tut_speech_recognition)


find_package(catkin REQUIRED COMPONENTS
  rospy
  cob_aalto_msgs
)

catkin_python_setup()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cogrob_tut_speech_recognition
   CATKIN_DEPENDS rospy cob_aalto_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/speech_recognition_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
