cmake_minimum_required(VERSION 2.8.3)
project(sdv_process)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FirebaseProcess.msg  
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS rospy
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  firebase_coms/firebase_launcher.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
