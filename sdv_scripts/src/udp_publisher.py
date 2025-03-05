#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This node receives UDP packages on default port (4210) and coverts them in 'udp'
messages, publishing in '/udp' topic.

Example of use:
rosrun sdv_scripts udp_publisher.py
'''
import socket
import threading
from signal import signal, SIGINT
import rospy
from sdv_scripts.msg import Udp

data = ''
data_stamp = 0.0
UDP_port = 4210
udp_listening = True
rate = 20


class UDP_Thread(threading.Thread):
    '''
    Class that runs a thread that listens for UDP packages, avoiding blocking 
    main thread while is waiting for a new message.
    '''

    def __init__(self):
        '''
        Configures UDP Socket, finding IP address.
        '''
        threading.Thread.__init__(self)

        # Setting UDP socket
        self.UDP_IP = get_ip()
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.UDP_IP, UDP_port))
        self.udp_socket.settimeout(0.2)

        # Print messages about UDP socket
        rospy.loginfo("Starting UDP socket")
        print("Starting UDP socket: ip={}, port={}".format(self.UDP_IP, UDP_port))

    def run(self):
        '''
        Entry point of thread. Enter in a loop while aplication is running, 
        listening for an UDP package. If package not arrives, saves 'stop' value
        in 'data' variable: this allows to stop aplication when a timeout 
        exception ocurs and main thread is finished.
        '''
        global udp_listening, data_stamp
        while udp_listening:
            global data
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                data_stamp = rospy.Time.now()
                datax = data
            except:
                data = ""


def get_ip():
    '''
    get_ip(): Returns machine IP
    '''
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


def sigint_handler(signal_received, frame):
    '''
    Allows to stop this aplication using Control + C keyboard combination.
    '''
    # Handle any cleanup here
    print('\nSIGINT or CTRL-C detected. Exiting...')
    global udp_listening
    udp_listening = False


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print("Parameter {} not defined. Defaulting to {}".format(name, default))
        return default


def publish_msg(data, udp_pub):
    if len(data) == 0:
        return  # empty message
    udp_msg = Udp()
    udp_msg.header.stamp = rospy.Time.now()
    udp_msg.content = data.decode("utf-8")
    udp_pub.publish(udp_msg)
    data = ""



# Main
if __name__ == '__main__':

    # Starting the node and configuring rate
    rospy.init_node('udp_publisher')
    udp_pub = rospy.Publisher('udp', Udp, queue_size=1)
    rate = rospy.Rate(rate)
    data_stamp = rospy.Time.now()

    # Print Exit message
    print("Press CTRL + C to exit...")

    # Starting UDP thread
    udp_thread = UDP_Thread()
    udp_thread.start()

    # Tell Python to run the sigint_handler() function when SIGINT is recieved
    signal(SIGINT, sigint_handler)

    # Loop: publishing Udp messages with defined rate
    while udp_listening:
        now_stamp = rospy.Time.now()
        publish_msg(data, udp_pub)
        rate.sleep()
    exit(0)
