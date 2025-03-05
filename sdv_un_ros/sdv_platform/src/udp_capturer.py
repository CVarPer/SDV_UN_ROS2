#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This node listens for an UDP message and takes a capture, using image messages 
from platform camera. UDP messages are published by udp_publisher.py
'''

import os
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from sdv_scripts.msg import Udp

camera_topic = '/camera/rgb/image_rect_color'
image = np.ones((100, 100,3), dtype=np.uint8)
last_capture_cmd_stamp = 0


def fetch_param(name, default):
    if rospy.has_param(name):
        print("Parameter {} defined. Setting to {}".format(name, rospy.get_param(name)))
        return rospy.get_param(name)
    else:
        print("Parameter {} not defined. Defaulting to {}".format(name, default))
        return default


def image_callback(msg):
    global image
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    

def udp_callback(msg):
    global last_capture_cmd_stamp, image
    data = msg.content
    if data == "capture":
        time_threshold = (rospy.Time.now() - last_capture_cmd_stamp).to_sec()
        if time_threshold > 1:
            index = 0
            files = [f for f in os.listdir('.') if os.path.isfile(f)]
            for f in files:
                if "capture_" in f:
                    i = f.replace('capture_', '')
                    i = i.replace('.png', '')
                    i = int(i)
                    if i > index:
                        index = i
            index += 1
            print("Making a capture with {} index".format(index))
            cv2.imwrite('capture_{}.png'.format(index), image)
            last_capture_cmd_stamp = rospy.Time.now()


rospy.init_node('udp_capturer')
camera_topic = fetch_param('~platform_camera_topic', camera_topic)
image_sub = rospy.Subscriber(camera_topic, Image, image_callback)
udp_sub = rospy.Subscriber('/udp', Udp, udp_callback)
last_capture_cmd_stamp = rospy.Time.now()
rospy.spin()
