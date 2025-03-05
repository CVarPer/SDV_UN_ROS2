#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This node reads UDP messages from '/udp' topic and then, publishes Twist nessages
in '/mobile_base/commands/velocity' topic. You can configure 
speed and acceleration using 'linear_scale' and 'angular_scale' parameters.

Parameters that you can configure are:
 - linear_scale: sets linear speed in m/s
 - angular_scale: sets angular speed in rad/s

Example of use:
rosrun sdv_scripts udp2twist.py _linear_scale:=1.0 _angular_scale:=1.0
'''

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sdv_scripts.msg import Udp

cmd_mapping = {  # 'code': [x, y, z]
    'f':    [ 1,  0,  0],
    'b':    [-1,  0,  0],
    'r':    [ 0, -1,  0],
    'l':    [ 0,  1,  0],
    'fr':   [ 1, -1,  0],
    'fl':   [ 1,  1,  0],
    'br':   [-1, -1,  0],
    'bl':   [-1,  1,  0],
    'tr':   [ 0,  0, -1],
    'tl':   [ 0,  0,  1],
    'stop': [ 0,  0,  0]}
g_vel_scales = [0.5, 0.5]
g_twist = None
rate = 20
stopped = True
data = ""


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print("Parameter {} not defined. Defaulting to {}".format(name, default))
        return default


def publish_msg(data, twist_pub):
    '''
    publish_msg(data, twist_pub): Search for command received in data and 
    publish a message in 'cmd_vel' topic using 'twist_pub' publisher.
    '''
    global g_twist, g_vel_scales, stopped
    if len(data) == 0 or not data in cmd_mapping:
        return  # unknown key
    if not stopped:
        vels = cmd_mapping[data]
        g_twist.linear.x = vels[0] * g_vel_scales[0]
        g_twist.linear.y = vels[1] * g_vel_scales[0]
        g_twist.angular.z = vels[2] * g_vel_scales[1]
        twist_pub.publish(g_twist)
    
    # If stopped, only send "stop" data once
    if data == "stop":
        stopped = True
    else:
        stopped = False


def udp_callback(msg):
    global data, data_stamp
    data_stamp = msg.header.stamp
    data = msg.content


# Main
if __name__ == '__main__':

    # Starting the node and configuring rate
    rospy.init_node('udp_to_twist')
    udp_sub = rospy.Subscriber('/udp', Udp, udp_callback)
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    g_twist = Twist()  # initializes to zero
    rate = rospy.Rate(rate)
    data_stamp = rospy.Time.now()

    # Fetching parameters
    g_vel_scales[0] = fetch_param('~linear_scale', g_vel_scales[1])
    g_vel_scales[1] = fetch_param('~angular_scale', g_vel_scales[0])

    # Print Exit message
    print("Press CTRL + C to exit...")

    # Loop: publishing Twist messages with defined rate
    while not rospy.is_shutdown():
        now_stamp = rospy.Time.now()
        dif = (now_stamp - data_stamp).to_sec()
        if dif > (0.2000):
            data = 'stop'
        publish_msg(data, twist_pub)
        rate.sleep()
    
    exit(0)
