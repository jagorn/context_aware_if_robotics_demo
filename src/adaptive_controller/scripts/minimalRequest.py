#! /usr/bin/env python
#
#  Author: Francesco Trapani
#
# Simple node simulating the request of tasks.

import rospy
import actionlib
from rosoclingoCommunication import RosoclingoCommunication


def execute():
    """
    Publishes on the  task request topic some simple task requests messages, and some after a fixed amount of time
    cancels some of the requests.
    """
    communication = RosoclingoCommunication()
    client = actionlib.ActionClient(communication.action_topic, communication.action)
    client.wait_for_server()

    rospy.sleep(1.0)

    request1 = 'at(sBot,4)'
    goal1 = client.send_goal(communication.goal(request1))
    rospy.loginfo('MinimalRequest - request %s sent\n', request1)
    rospy.sleep(2.0)

    request2 = 'at(sBot,2)'
    goal2 = client.send_goal(communication.goal(request2))
    rospy.loginfo('MinimalRequest - request %s sent\n', request2)
    rospy.sleep(2.0)

    request3 = 'at(sBot,5)'
    goal3 = client.send_goal(communication.goal(request3))
    rospy.loginfo('MinimalRequest - request %s sent\n', request3)
    rospy.sleep(2.0)

    goal3.cancel()
    rospy.loginfo('MinimalRequest - request %s canceled\n', request3)
    rospy.sleep(2.0)

    goal2.cancel()
    rospy.loginfo('MinimalRequest - request %s canceled\n', request2)
    rospy.sleep(2.0)

# ROS node
if __name__ == '__main__':
    try:
        rospy.init_node('minimal_request')
        execute()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")