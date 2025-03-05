cmake_minimum_required(VERSION 2.8.3)
project(sdv_scripts)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Declare ROS messages and services
add_message_files(
  FILES 
    Udp.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    message_runtime
)

