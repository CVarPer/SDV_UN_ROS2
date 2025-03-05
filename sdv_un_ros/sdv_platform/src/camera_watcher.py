#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This node uses images from platform camera and publish a message if in the platform
there is an object. Also, publishes user feedback for LED an buzzer for a quick
response.
'''

from time import sleep
import rospy
import cv2
import cv_bridge
import numpy as np
from image_recog_tools import get_mask, ImageClassifier, resize_img

from sensor_msgs.msg import Image
from sdv_msgs.msg import SdvPlatform
from sdv_msgs.msg import Buzzer
from sdv_msgs.msg import LED

global camera_topic, platform_status_topic, buzzer_topic, led_topic
global platform_status_values, show_samples, image_size
camera_topic = '/camera/rgb/image_rect_color'
platform_status_topic = '/platform'
buzzer_topic = '/buzzer'
led_topic = '/led'
platform_status_values = {'undefined':'undefined', 'loaded':'loaded', 'empty':'empty'}
show_samples = False
image_size = 100


def fetch_param(name, default):
    if rospy.has_param(name):
        print("Parameter {} defined. Setting to {}".format(name, rospy.get_param(name)))
        return rospy.get_param(name)
    else:
        print("Parameter {} not defined. Defaulting to {}".format(name, default))
        return default


class PlatformWatcher:


    def __init__(self):
        global camera_topic, platform_status_topic, platform_status_values
        self.last_image = None
        self.mask = None
        self.roi = None
        self.obtained_roi = False
        self.platform_status = platform_status_values['undefined']
        self.last_platform_status = platform_status_values['undefined']
        self.last_platform_status_stamp = None
        self.image_classifier = None
        self.image_counter = 0

        # Configuring subscribers and publishers
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)
        self.platform_pub = rospy.Publisher(platform_status_topic, SdvPlatform, queue_size=10)
        self.buzzer_pub = rospy.Publisher(buzzer_topic, Buzzer, queue_size=10)
        self.led_pub = rospy.Publisher(led_topic, LED, queue_size=10)

        self.led_feedback()
        self.get_roi()


    def image_callback(self, msg):
        global platform_status_values, show_samples, image_size

        self.last_image = msg
        self.image_counter += 1
        if self.obtained_roi == False:
            return

        if self.image_counter < 15:
            return
        else:
            self.image_counter = 0

        # Detects objetc
        image = self.bridge.imgmsg_to_cv2(msg)
        image = resize_img(image, image_size)
        [res, img_masked, dur] = self.image_classifier.get_object(image) 
        
        if show_samples:
            masked = np.array(img_masked, 'uint8')
            cv2.namedWindow("mask", 1)
            cv2.namedWindow("object", 2)
            cv2.imshow("mask", self.mask)
            cv2.imshow("object", masked)
            cv2.waitKey(3)
            print("Task duration = {}, object = {}".format(dur, res))

        # Setting platform status
        current_status = platform_status_values['undefined']
        now_stamp = rospy.Time.now()
        if res == 1:
            current_status = platform_status_values['loaded']
        else:
            current_status = platform_status_values['empty']

        # Store time stamp when platform status changes
        if self.last_platform_status != current_status:
            self.last_platform_status = current_status
            self.last_platform_status_stamp = now_stamp

        # Set platform status if last change ocurred at least 3 seconds ago
        dur = now_stamp - self.last_platform_status_stamp
        dur = dur.to_sec()
        if dur >= 2.0:
            if self.platform_status == platform_status_values['empty'] and current_status == platform_status_values['loaded']:
                self.buzzer_feedback(150, 100, 3)
            self.platform_status = current_status
            #self.pub_platform_status()


    def get_roi(self):

        global image_size
        while self.last_image == None and not rospy.is_shutdown():
            print("Waiting for an image...")
            sleep(2)

        while not self.obtained_roi:

            # Color space
            image = self.bridge.imgmsg_to_cv2(self.last_image)
            image = resize_img(image, image_size)

            # Get mask
            [roi_detected, mask, approx_poly, centroids] = get_mask(image)

            # Get image classifier
            self.image_classifier = ImageClassifier(mask)

            # Asigning values
            self.mask = mask
            self.obtained_roi = roi_detected
            self.last_platform_status_stamp = rospy.Time.now()

            if self.obtained_roi:
                print("ROI generated")
                print(self.last_platform_status_stamp)
                self.buzzer_feedback(250, 250, 2)
            else:
                print("Nothing detected")
                sleep(2)

    
    def pub_platform_status(self):
        # Setting SdvPlatform data and publishing it
        plat_status_msg = SdvPlatform()
        plat_status_msg.header.stamp = rospy.Time.now()
        plat_status_msg.status = self.platform_status
        plat_status_msg.source = 'camera'

        # Publish msg
        self.platform_pub.publish(plat_status_msg)

        # Publish LED feedback
        self.led_feedback()


    def buzzer_feedback(self, time_on, time_off, cicles):
        # Setting Buzzer data and publishig it
        buzzer_msg = Buzzer()
        buzzer_msg.header.stamp = rospy.Time.now()
        buzzer_msg.time_on = time_on
        buzzer_msg.time_off = time_off
        buzzer_msg.cicles = cicles
        self.buzzer_pub.publish(buzzer_msg)
        return

    def led_feedback(self):
        # Setting LED values
        l_values = [0, 0, 0]
        if self.platform_status == platform_status_values['loaded']:
            l_values = [0, 255, 0]
        if self.platform_status == platform_status_values['empty']:
            l_values = [255, 0, 0]

        # Setting LED msg
        led_msg = LED()
        led_msg.header.stamp = rospy.Time.now()
        led_msg.red = l_values[0]
        led_msg.green = l_values[1]
        led_msg.blue = l_values[2]

        # Publish LED msg
        self.led_pub.publish(led_msg)
        return


rospy.init_node('camera_watcher')
camera_topic = fetch_param('~platform_camera_topic', camera_topic)
platform_status_topic = fetch_param('~platform_status_topic', platform_status_topic)
show_samples = fetch_param('~show_samples', show_samples)
watcher = PlatformWatcher()
rate = rospy.Rate(2)

while not rospy.is_shutdown():
    watcher.pub_platform_status()
    rate.sleep()
