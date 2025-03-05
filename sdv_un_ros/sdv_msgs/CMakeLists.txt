cmake_minimum_required(VERSION 2.8.3)
project(sdv_msgs)

# Load and setup packages
find_package(
  catkin REQUIRED 
  COMPONENTS
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
)

## Declare ROS messages and services
add_message_files(
  FILES 
    Batteries.msg
    Battery.msg
    Buzzer.msg
    Drivers.msg
    Encoder.msg
    Flexiforce.msg
    FourMotors.msg
    ImuRaw.msg
    LED.msg
    MotorDriver.msg
    PanelButton.msg
    SdvPlatform.msg
    SdvStatus.msg
    TagRfid.msg
    TwoMotors.msg
    Ultrasound.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

# Setup catkin packages
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
)