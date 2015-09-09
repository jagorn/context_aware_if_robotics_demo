#! /usr/bin/env python
#
#  Author: Francesco Trapani
#
# Simple node simulating the request of tasks.

import rospy
import actionlib
import random
from rosoclingoCommunication import RosoclingoCommunication


def execute():
    """
    Publishes on the  task request topic some simple task requests messages
    """
    robot_name = rospy.get_param("robot_name", "robot0")
    communication = RosoclingoCommunication(robot_name)

    client = actionlib.ActionClient(communication.action_topic, communication.action)
    client.wait_for_server()

    request = 'at('+robot_name+',flag_4)'
    client.send_goal(communication.goal(request))
    rospy.loginfo('MinimalRequest - request %s sent\n', request)

# ROS node
if __name__ == '__main__':

    try:
        rospy.init_node('minimal_request')
        execute()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")