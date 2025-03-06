cmake_minimum_required(VERSION 3.5)
project(sdv_process)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)  

# Install Python scripts
install(PROGRAMS
  firebase_coms_launcher.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclpy std_msgs action_msgs geometry_msgs nav2_msgs)  
# Required for ament
ament_package()

