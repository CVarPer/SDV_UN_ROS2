#include <string.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <motor/four_drive_controller.h>
#include <motor/motor.h>
#include <sdv_msgs/FourMotors.h>

#define _USE_MATH_DEFINES // for C++
#include <cmath>

using namespace Eigen;
using namespace std;

FourDriveController::FourDriveController()
{
    back_left_motor = new Motor(wheel_radius, false);
    back_right_motor = new Motor(wheel_radius, true);
    front_left_motor = new Motor(wheel_radius, false);
    front_right_motor = new Motor(wheel_radius, true);

    motors[0] = back_left_motor;
    motors[1] = back_right_motor;
    motors[2] = front_left_motor;
    motors[3] = front_right_motor;
}

string FourDriveController::getCommandString(
    geometry_msgs::Vector3 linear, 
    geometry_msgs::Vector3 angular)
{
    string cmd = "";

    // Inverse Kineamatics of Mecanum Plaform. Using extra variables for simplicity
    double a = wheel_axis_separation / 2.0;
    double b = wheel_separation / 2.0;
    double r = wheel_radius;

    VectorXd v(3);
    v(0) = linear.x;
    v(1) = linear.y;
    v(2) = angular.z;

    MatrixXd m(4, 3);
    m << 1, -1, -(a + b),
         1,  1,  (a + b),
         1,  1, -(a + b),
         1, -1,  (a + b);

    VectorXd w_wheels(4);
    w_wheels << (1/r) * m * v;

    // Converting from rad/s to rps
    w_wheels << w_wheels / (2 * M_PI);

    // Storing values
    front_left_motor->setRefSpeed(w_wheels(0));
    front_right_motor->setRefSpeed(w_wheels(1));
    back_left_motor->setRefSpeed(w_wheels(2));
    back_right_motor->setRefSpeed(w_wheels(3));

    // Generate command string for Tiva board
    cmd = "m 1 " + std::to_string(back_left_motor->getRefSpeedRps())
           + " " + std::to_string(back_right_motor->getRefSpeedRps())
           + " " + std::to_string(front_left_motor->getRefSpeedRps())
           + " " + std::to_string(front_right_motor->getRefSpeedRps())
           + "\r";

    return cmd;
}

void FourDriveController::setWheelSeparation(double distance)
{
    wheel_separation = distance;
    for(uint8_t i = 0; i < 4; i++)
    {
        motors[i]->setWheelSeparation(distance);
    }
}

void FourDriveController::setAxisWheelSeparation(double distance)
{
    wheel_axis_separation = distance;
    for(uint8_t i = 0; i < 4; i++)
    {
        motors[i]->setAxisWheelSeparation(distance);
    }
}

void FourDriveController::setNodeHandle(ros::NodeHandle *nh)
{
    FourDriveController::nh = nh;
    ref_speed_pub = nh->advertise<sdv_msgs::FourMotors>("/motors/odom/four_motors/ref_speed", 20);
    actual_speed_pub = nh->advertise<sdv_msgs::FourMotors>("/motors/odom/four_motors/actual_speed", 20);
}

void FourDriveController::setActualSpeeds(double speeds[]) 
{
    for(uint8_t i = 0; i < 4; i++)
    {
        motors[i]->setActualSpeed(speeds[i]);
    }
}

void FourDriveController::publishMotorRefSpeeds()
{
    // Message to publish in motors topic
    sdv_msgs::FourMotors motors_msg;
    motors_msg.header.stamp = ros::Time::now();
    motors_msg.back_left_rps = back_left_motor->getRefSpeedRps();
    motors_msg.back_right_rps = back_right_motor->getRefSpeedRps();
    motors_msg.front_left_rps = front_left_motor->getRefSpeedRps();
    motors_msg.front_right_rps = front_right_motor->getRefSpeedRps();
    ref_speed_pub.publish(motors_msg);
}

void FourDriveController::publishMotorActualSpeeds()
{
    // Message to publish in motors topic
    sdv_msgs::FourMotors motors_msg;
    motors_msg.header.stamp = ros::Time::now();
    motors_msg.back_left_rps = back_left_motor->getActualSpeedRps();
    motors_msg.back_right_rps = back_right_motor->getActualSpeedRps();
    motors_msg.front_left_rps = front_left_motor->getActualSpeedRps();
    motors_msg.front_right_rps = front_right_motor->getActualSpeedRps();
    actual_speed_pub.publish(motors_msg);
}