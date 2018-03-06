cmake_minimum_required(VERSION 2.8.3)
project(cogrob_tut_msgs)

find_package(catkin REQUIRED
  rospy
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
)

add_message_files(
    FILES
    Action.msg
    InfoSymbol.msg
    Target.msg
    Request.msg
    Instruction.msg
    Person.msg
)

add_service_files(
    FILES
    retrieveActions.srv
    retrieveTargets.srv
    retrieveRequests.srv
    retrievePeople.srv
    submitRequest.srv
    query.srv
)

add_action_files(
   FILES
   AddInfoConversation.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cogrob_tut_msgs
   CATKIN_DEPENDS rospy std_msgs geometry_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)
