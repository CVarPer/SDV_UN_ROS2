#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This node takes a image from platform camera and show it in a window.
'''

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

camera_topic = '/camera/rgb/image_rect_color'

def fetch_param(name, default):
    if rospy.has_param(name):
        print("Parameter {} defined. Setting to {}".format(
            name, rospy.get_param(name)))
        return rospy.get_param(name)
    else:
        print("Parameter {} not defined. Defaulting to {}".format(name, default))
        return default


class ImageWindower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("rgb", 1)
        #cv2.namedWindow("ir", 2)
        self.image_rgb_sub = rospy.Subscriber(
            camera_topic, Image, self.image_rgb_callback)
        #self.image_ir_sub = rospy.Subscriber('/camera/ir/image_rect_ir', Image, self.image_ir_callback)

    def image_rgb_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        cv2.imshow("rgb", image)
        cv2.waitKey(3)

    def image_ir_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("ir", image)
        cv2.waitKey(3)

rospy.init_node('camera_window_samples')
camera_topic = fetch_param('~platform_camera_topic', camera_topic)
window = ImageWindower()
rospy.spin()
