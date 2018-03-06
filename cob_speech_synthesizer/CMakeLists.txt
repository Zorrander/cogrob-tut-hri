cmake_minimum_required(VERSION 2.8.3)
project(cob_speech_synthesizer)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
    std_msgs
    cob_script_server
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cob_speech_synthesizer
	CATKIN_DEPENDS rospy std_msgs cob_script_server
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/speech_synthesizer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
