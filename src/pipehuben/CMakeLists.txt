cmake_minimum_required(VERSION 2.8.3)
project(pipehuben)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  geometry_msgs
  std_msgs
  message_generation
)

add_action_files(
  FILES
  MoveBase.action
)

add_service_files(
  FILES
  StartCharge.srv
  FindWarn.srv
  Detection_srv.srv
  Order.srv
  yt_data.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    message_runtime
    actionlib
    actionlib_msgs
    std_msgs
    geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
