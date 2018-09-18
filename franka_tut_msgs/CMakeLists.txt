cmake_minimum_required(VERSION 2.8.3)
project(franka_tut_msgs)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  geometry_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  Instruction.msg
  Instructions.msg
  Action.msg
  Gripper.msg
  Arm.msg
)

add_action_files(
  DIRECTORY action
  FILES
  ExecuteTask.action
  RecordMotion.action
  AddInfoConversation.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES franka_tut_msgs
   CATKIN_DEPENDS actionlib_msgs
#  DEPENDS system_lib
)
