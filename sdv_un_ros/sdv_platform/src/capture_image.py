#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This node takes a image from platform camera and saves it in a png file in the 
current working directory.
'''

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image

camera_topic = '/camera/rgb/image_rect_color'
image_sub = None

def fetch_param(name, default):
    if rospy.has_param(name):
        print("Parameter {} defined. Setting to {}".format(name, rospy.get_param(name)))
        return rospy.get_param(name)
    else:
        print("Parameter {} not defined. Defaulting to {}".format(name, default))
        return default

def image_callback(msg):
    global image_sub
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg)
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    cv2.imwrite('capture.png', rgb)
    image_sub.unregister()
    print('Capture saved in a file named "capture.png". Exiting.')
    rospy.signal_shutdown("Task completed")


rospy.init_node('platform_watcher')
camera_topic = fetch_param('~platform_camera_topic', camera_topic)
print('Waiting for image msg...')
image_sub = rospy.Subscriber(camera_topic, Image, image_callback)
rospy.spin()
