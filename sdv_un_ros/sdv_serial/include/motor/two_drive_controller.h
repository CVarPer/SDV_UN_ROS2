#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <motor/motor.h>

#pragma once

using namespace std;

class TwoDriveController
{

public:
    TwoDriveController();
    void setWheelSeparation(double distance);
    void setAxisWheelSeparation(double distance);
    string getCommandString(geometry_msgs::Vector3 vel_linear, geometry_msgs::Vector3 vel_angular);
    void setNodeHandle(ros::NodeHandle *nh);
    void setActualSpeeds(double speeds[]);
    void publishMotorRefSpeeds();
    void publishMotorActualSpeeds();

private:
    Motor *left_motor;
    Motor *right_motor;
    double r = 0.075;
    double B = 0.44010;
    double N = 3.2 * 4;
    double wL;
    double wR;
    
    ros::NodeHandle *nh;
    ros::Publisher ref_speed_pub;
    ros::Publisher actual_speed_pub;
};