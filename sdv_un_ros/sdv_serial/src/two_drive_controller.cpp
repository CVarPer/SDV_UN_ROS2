#include <string.h>
#include <tools.h>
#include <geometry_msgs/Vector3.h>
#include <motor/two_drive_controller.h>
#include <motor/motor.h>
#include <sdv_msgs/TwoMotors.h>

using namespace std;

TwoDriveController::TwoDriveController()
{
    left_motor = new Motor(r, false);
    right_motor = new Motor(r, true);
}

string TwoDriveController::getCommandString(
    geometry_msgs::Vector3 vel_linear,
    geometry_msgs::Vector3 vel_angular)
{
    string cmd;

    // Get PWM percents of every wheel
    wL = left_motor->getPwmPercent(vel_linear.x, vel_angular.z);
    wR = right_motor->getPwmPercent(vel_linear.x, vel_angular.z);

    // Constraining wheel speeds
    wL = tools::constrain(wL, -40.0, 40.0);
    wR = tools::constrain(wR, -40.0, 40.0);

    // Storing values
    left_motor->setRefSpeed(wL);
    right_motor->setRefSpeed(wR);

    // Generaring message string
    cmd = "m 1 " + std::to_string(int(wL)) + " " + std::to_string(int(wR)) + "\r";

    return cmd;
}

void TwoDriveController::setWheelSeparation(double distance)
{
    left_motor->setWheelSeparation(distance);
    right_motor->setWheelSeparation(distance);
}

void TwoDriveController::setAxisWheelSeparation(double distance)
{
    left_motor->setAxisWheelSeparation(distance);
}

void TwoDriveController::setNodeHandle(ros::NodeHandle *nh)
{
    TwoDriveController::nh = nh;
    ref_speed_pub = nh->advertise<sdv_msgs::TwoMotors>("/motors/odom/two_motors/ref_speed", 20);
    actual_speed_pub = nh->advertise<sdv_msgs::TwoMotors>("/motors/odom/two_motors/actual_speed", 20);
}

void TwoDriveController::setActualSpeeds(double speeds[]) 
{
    left_motor->setActualSpeed(speeds[0]);
    right_motor->setActualSpeed(speeds[1]);
}

void TwoDriveController::publishMotorRefSpeeds()
{
    // Message to publish in motors topic
    sdv_msgs::TwoMotors motors_msg;
    motors_msg.header.stamp = ros::Time::now();
    motors_msg.left = left_motor->getRefSpeedRps();
    motors_msg.right = right_motor->getRefSpeedRps();
    ref_speed_pub.publish(motors_msg);
}

void TwoDriveController::publishMotorActualSpeeds()
{
    // Message to publish in motors topic
    sdv_msgs::TwoMotors motors_msg;
    motors_msg.header.stamp = ros::Time::now();
    motors_msg.left = left_motor->getActualSpeedRps();
    motors_msg.right = right_motor->getActualSpeedRps();
    actual_speed_pub.publish(motors_msg);
}