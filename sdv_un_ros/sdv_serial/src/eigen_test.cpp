#include <unistd.h>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <inttypes.h>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sdv_msgs/FourMotors.h>

using namespace Eigen;
using namespace std;

// Prototypes
void move_motors_callback(const geometry_msgs::Twist &cmd);

// Variables
ros::Publisher motors_pub;

// Functions
void move_motors_callback(const geometry_msgs::Twist &cmd)
{
    // geometry_msgs::Vector3 vel_linear;
    // geometry_msgs::Vector3 vel_angular;
    // vel_linear = cmd.linear;
    // vel_angular = cmd.angular;

    double a = 0.15;
    double b = 0.12;
    double r = 0.100;

    VectorXd v(3);
    v(0) = cmd.linear.x;
    v(1) = cmd.linear.y;
    v(2) = cmd.angular.z;

    MatrixXd m(4, 3);
    m << 1, -1, -(a + b),
         1,  1,  (a + b),
         1,  1, -(a + b),
         1, -1,  (a + b);

    VectorXd w_wheels(4);
    w_wheels << (1/r) * m * v;

    cout << w_wheels << "\n\n";

    sdv_msgs::FourMotors motors_msg;
    motors_msg.header.stamp = ros::Time::now();
    motors_msg.front_left_rps = w_wheels(0);
    motors_msg.front_right_rps = w_wheels(1);
    motors_msg.back_left_rps = w_wheels(2);
    motors_msg.back_right_rps = w_wheels(3);
    motors_pub.publish(motors_msg);
}

int main(int argc, char **argv)
{
    // Configuring node
    ros::init(argc, argv, "eigen_test_node");
    ros::NodeHandle nh("~");

    // Publishers
    motors_pub = nh.advertise<sdv_msgs::FourMotors>("/motors/four_motors", 20);

    // Subscribing to topics
    ros::Subscriber velocity_subs = nh.subscribe("/mobile_base/commands/velocity", 20, move_motors_callback);

    // Configuring ROS loop rate
    ros::Rate loop_rate(2);

    // ROS Loop
    while (ros::ok())
    {
        // Spin
        ros::spinOnce();
        loop_rate.sleep();

    } // End of ROS Loop
}