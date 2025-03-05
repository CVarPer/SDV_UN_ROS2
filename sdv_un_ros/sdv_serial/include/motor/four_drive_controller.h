#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <motor/motor.h>

#pragma once

using namespace std;

class FourDriveController
{

public:
    FourDriveController();
    void setWheelSeparation(double distance);
    void setAxisWheelSeparation(double distance);
    string getCommandString(geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular);
    void setNodeHandle(ros::NodeHandle *nh);
    void setActualSpeeds(double speeds[]);
    void publishMotorRefSpeeds();
    void publishMotorActualSpeeds();

private:
    Motor *back_left_motor;
    Motor *back_right_motor;
    Motor *front_left_motor;
    Motor *front_right_motor;
    Motor *motors[4];
    double wheel_radius = 0.05;
    double wheel_separation = 0.415;
    double wheel_axis_separation = 0.39;
    double N = 3.2 * 4;

    ros::NodeHandle *nh;
    ros::Publisher ref_speed_pub;
    ros::Publisher actual_speed_pub;
    
};
