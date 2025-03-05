#!/usr/bin/env python

import json
from time import sleep
import rospy
from actionlib import SimpleActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sdv_msgs.msg import SdvPlatform


def fetch_param(name, default):
    if rospy.has_param(name):
        print("Parameter {} defined. Setting to {}".format(name, rospy.get_param(name)))
        return rospy.get_param(name)
    else:
        print("Parameter {} not defined. Defaulting to {}".format(name, default))
        return default


class BridgeConection():

    def __init__(self):

        # Configure node
        rospy.init_node('firebase_listener', log_level=rospy.DEBUG, anonymous=True)

        # Fetching global parameters
        robot_model = fetch_param('/sdv/robot_model', "SDV1")
        
        # Fetching private parameters
        self.namespace = fetch_param('~namespace', "/")
        self.robot_name = fetch_param('~robot_name', robot_model)

        # Obtains SDV suffix from robot_name field
        self.suffix = int( self.robot_name[len(self.robot_name) - 1] )

        # Formating namespace
        self.namespace = self.namespace.replace(" ", "_")
        if self.namespace != "/":
            # Appending prefix
            i = self.namespace[0]
            if i != "/":
                self.namespace = "/" + self.namespace
            # Appending suffix
            f = self.namespace[len(self.namespace) - 1]
            if f != "/":
                self.namespace = self.namespace + "/"
        rospy.logdebug("Full namespace: {}".format(self.namespace))

        # Creates a publisher
        self.resultPub = rospy.Publisher('step_result', String, queue_size=10)

        # Subscribing to step_command topic
        rospy.Subscriber('step_command', String, self.step_received)

        # Subscribing to platform topic
        platform_topic = '{}platform'.format(self.namespace)
        print('Platform Topic: {}'.format(platform_topic))
        rospy.Subscriber(platform_topic, SdvPlatform, self.platform_status)
        self.plaform_status_last_msg = SdvPlatform()
        self.plaform_status_last_msg.header = rospy.Time.now()
        self.plaform_status_last_msg.status = "undefined"

        # Spin
        rospy.spin()

    def step_received(self, msg):

        # Process received data
        jsn = json.loads(msg.data)
        rospy.logdebug('Decoded received step to %s' % jsn)
        stepargs = jsn['args']

        # Drops received json if suffix is diferent from defined SDV suffix
        if stepargs['suffix'] != self.suffix:
            rospy.logdebug("Received suffix don't match with defined suffix, {}. Dropping message.".format(self.suffix))
            return
        else:
            rospy.logdebug("Received suffix match with defined suffix, {}".format(self.suffix))

        # Creating action client
        client = SimpleActionClient(self.namespace + 'move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = stepargs['x']
        goal.target_pose.pose.position.y = stepargs['y']
        goal.target_pose.pose.orientation.z = stepargs['z']
        goal.target_pose.pose.orientation.w = stepargs['w']

        # If defined, wait for platform trigger
        if 'trigger' in stepargs:
            platform_trigger = stepargs['trigger']
        else:
            platform_trigger = 'none'
        if platform_trigger != "none":
            while self.plaform_status_last_msg.status != "loaded":
                sleep(1)
                continue

        # Publish action
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            result = {
                'success': False,
                'error': 'Action server error'
            }
        else:
            result = client.get_state()
            rospy.logwarn('wait %s, result: %s' % (wait, result))
            success = result == GoalStatus.SUCCEEDED
            result = {
                'success': success,
                'error': None if success else client.get_goal_status_text()
            }

        self.resultPub.publish(json.dumps(result, ensure_ascii=False))

    def platform_status(self, msg):
        self.plaform_status_last_msg = msg
        #print(msg.status)
        return

if __name__ == '__main__':
    BridgeConection()
