#!/usr/bin/env python3

from yaml.error import Mark
import rospy
import math
import numpy as np
from rospy import Subscriber
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

rate = 100
mag_listener: Subscriber = None


def mag_listener(msg: MagneticField):

    # Normalizing vector
    mag_bias_x = 0.00002115550
    mag_bias_y = 0.00001905450
    mag_bias_z = 0.00002745700

    mag_vec = np.array([
        msg.magnetic_field.x - mag_bias_x, 
        msg.magnetic_field.y - mag_bias_y, 
        msg.magnetic_field.z - mag_bias_z])
    m = math.sqrt(math.pow(mag_vec[0], 2) + math.pow(mag_vec[1], 2) + math.pow(mag_vec[2], 2))
    mag_norm = mag_vec / m * 0.1

    # Building message
    marker_msg: Marker = Marker()
    marker_msg.action = Marker.MODIFY
    marker_msg.type = Marker.ARROW
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.header.frame_id = "imu"
    marker_msg.pose.orientation.x = 0
    marker_msg.pose.orientation.y = 0
    marker_msg.pose.orientation.z = 0
    marker_msg.pose.orientation.w = 1

    tail: Point = Point()
    tail.x = 0
    tail.y = 0
    tail.z = 0

    head: Point = Point()
    head.x = mag_norm[0]
    head.y = mag_norm[1]
    head.z = -mag_norm[2]

    marker_msg.points = [tail, head]

    marker_msg.scale.x = 0.01
    marker_msg.scale.y = 0.015

    marker_msg.color.r = 0.0
    marker_msg.color.g = 1.0
    marker_msg.color.b = 1.0
    marker_msg.color.a = 1.0
    
    # Publishing message
    marker_pub.publish(marker_msg)


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print("parameter {} not defined. Defaulting to {}".format(name, default))
        return default

if __name__ == '__main__':

    # Configuring node
    rospy.init_node('mag2marker')
    marker_pub = rospy.Publisher('/vis/marker', Marker, queue_size=1)
    mag_listener = rospy.Subscriber('/imu/mag', MagneticField, mag_listener)
    rate = rospy.Rate(rate)

    # Fetching parameters
    # show = fetch_param('~show', False)

    # Loop
    while not rospy.is_shutdown():

        # Sending Twist message and sleeping
        rate.sleep()