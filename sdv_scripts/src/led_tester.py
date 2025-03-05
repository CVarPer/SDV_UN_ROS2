#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sdv_msgs.msg import LED

rate = 10.0
red = 0
green = 0
blue = 0
led_pub = None
steps = [0, 0, 0]
current_step = 0
total_steps = rate * 5
current_trans = 0
transitions = {
    "black_to_red":[0, 9, 1],
    "red_to_green":[1, 1, 2], 
    "green_to_blue":[2, 2, 3],
    "blue_to_red":[3, 3, 1],
    "red_to_cyan":[4, 1, 4],
    "cyan_to_purple":[5, 4, 5],
    "purple_to_yellow":[6, 5, 6],
    "yellow_to_orange":[7, 6, 7],
    "orange_to_white":[8, 7, 8],
    "white_to_black":[9, 8, 9],
    "balck_to_black":[10, 9, 9],
    }
colors = {
    "red":[1, 255, 0, 0],
    "green":[2, 0, 255, 0],
    "blue":[3, 0, 0, 255],
    "cyan":[4, 0, 255, 255],
    "purple":[5, 255, 0, 255],
    "yellow":[6, 255, 255, 0],
    "orange":[7, 255, 165, 0],
    "white":[8, 255, 255, 255],
    "black":[9, 0, 0, 0],
}


def sendLedMessage(r, g, b):
    global led_pub
    led_msg = LED()
    led_msg.red = r
    led_msg.green = g
    led_msg.blue = b
    led_pub.publish(led_msg)


def getSteps():
    global red, green, blue, transitions, colors, current_trans, steps, total_steps
    init_color = None
    final_color = None
    init_color_rgb = None
    final_color_rgb = None

    # Search transitions index in the array
    for f in transitions.keys():
        if transitions[f][0] == current_trans:
            init_color = transitions[f][1]
            final_color = transitions[f][2]
    
    # Search for color values
    for k in colors.keys():
        if colors[k][0] == init_color:
            init_color_rgb = [colors[k][1], colors[k][2], colors[k][3]]
        if colors[k][0] == final_color:
            final_color_rgb = [colors[k][1], colors[k][2], colors[k][3]]

    # Generate steps
    for i in range(0, 3):
        steps[i] = (final_color_rgb[i] - init_color_rgb[i]) / total_steps


def set_current_step():
    global red, green, blue, steps, led_pub

    # Publish message
    led_msg = LED()
    led_msg.header.stamp = rospy.Time.now()
    led_msg.red = int(red)
    led_msg.green = int(green)
    led_msg.blue = int(blue)
    led_pub.publish(led_msg)

    # Update values
    red += steps[0]
    green += steps[1]
    blue += steps[2]

    #print("r = {}, g = {}, b = {}".format(red, green, blue))
    

if __name__ == '__main__':

    # Configuring node
    rospy.init_node('led_tester_node')
    led_pub = rospy.Publisher('/led', LED, queue_size=5)
    rate = rospy.Rate(rate)
    getSteps()

    # Loop
    while not rospy.is_shutdown():

        # Send LED message using the current transtion steps
        set_current_step()
        current_step += 1

        # Check if current transitions is completed
        if current_step >= total_steps:
            current_trans += 1
            if current_trans >= len(transitions):
                current_trans = 0
            getSteps()
            current_step = 0
        
        # Sleep to complete the rate
        rate.sleep()
