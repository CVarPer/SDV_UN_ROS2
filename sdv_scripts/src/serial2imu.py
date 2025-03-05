#!/usr/bin/env python3

import rospy
import math
import serial
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

port = "/dev/ttyACM0"
baudrate = 115200
rate = 100

def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print("parameter {} not defined. Defaulting to {}".format(name, default))
        return default

if __name__ == '__main__':

    # Configuring node
    rospy.init_node('serial2imu')
    imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=1)
    mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=1)
    rate = rospy.Rate(rate)

    # Fetching parameters
    port = fetch_param('~port', port)
    show = fetch_param('~show', False)

    # Serial handler
    ser = serial.Serial(port, baudrate)  # open serial port

    # Loop
    while not rospy.is_shutdown():

        line = str(ser.readline())   # read a '\n' terminated line
        line = line.lstrip("b")
        line = line.strip("'")
        line = line.rstrip("\\n")

        values = line.split()

        if len(values) == 14:

            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu"
            imu_msg.linear_acceleration_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
            imu_msg.angular_velocity_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
            imu_msg.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
            
            mag_msg = MagneticField()
            mag_msg.header.stamp = rospy.Time.now()
            mag_msg.header.frame_id = "imu"
            
            imu_msg.linear_acceleration.x = float(values[1])
            imu_msg.linear_acceleration.y = float(values[2])
            imu_msg.linear_acceleration.z = float(values[3])

            imu_msg.angular_velocity.x = float(values[4])
            imu_msg.angular_velocity.y = float(values[5])
            imu_msg.angular_velocity.z = float(values[6])

            mag_msg.magnetic_field.x = float(values[7])
            mag_msg.magnetic_field.y = float(values[8])
            mag_msg.magnetic_field.z = float(values[9])

            imu_msg.orientation.x = float(values[10])
            imu_msg.orientation.y = float(values[11])
            imu_msg.orientation.z = float(values[12])
            imu_msg.orientation.w = float(values[13])

            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)

            if show:
                print(line.replace(values[0], ""))
        else:
            print(str(rospy.Time.now().to_sec()) + ": Not enough values in serial line")

        # Sending Twist message and sleeping
        rate.sleep()