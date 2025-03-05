cmake_minimum_required(VERSION 2.8.3)
project(sdv_serial)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED 
  COMPONENTS
    roscpp
    serial
    sdv_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    sdv_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

## sdv_serial_node
add_executable(
  sdv_serial_node 
    src/sdv_serial_node.cpp 
    src/motor.cpp
    src/two_drive_controller.cpp
    src/four_drive_controller.cpp
    src/tools.cpp
)
target_link_libraries(sdv_serial_node ${catkin_LIBRARIES})
add_dependencies(sdv_serial_node sdv_msgs_generate_messages_cpp)

add_executable(
  eigen_test_node 
    src/eigen_test.cpp
)
target_link_libraries(eigen_test_node ${catkin_LIBRARIES})
add_dependencies(eigen_test_node sdv_msgs_generate_messages_cpp)

add_executable(
  args_test_node
    src/args_test.cpp 
    src/tools.cpp
)
target_link_libraries(args_test_node ${catkin_LIBRARIES})