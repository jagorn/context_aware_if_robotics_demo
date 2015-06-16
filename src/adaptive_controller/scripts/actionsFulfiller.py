#! /usr/bin/env python
#
#  Author: Francesco Trapani
#
# Simple node simulating the fulfilment of actions.
# For each ActionRequest message received, the node will reply after a fixed amount of time with
# an ActionFeedback message with positive feedback.

import rospy
from rosoclingoCommunication import RosoclingoCommunication

__communication = RosoclingoCommunication()
__topic = rospy.Publisher(__communication.in_topic, __communication.in_message, latch=True, queue_size=10)


def __on_action_request(message):
    """
    Callback for ActionRequest messages reception:
    Creates and publishes a positive feedback ActionFeedback messages with same 'action' and 'robot' of the request.
    """

    # Create the message
    feedback_msg = __communication.in_message()
    feedback_msg.robot = message.robot
    feedback_msg.action = message.action
    feedback_msg.value = "success"

    # Submit
    rospy.sleep(2.5)
    __topic.publish(feedback_msg)
    rospy.loginfo("ActionsFulfiller - action %s:%s executed successfully\n", message.robot, message.action)

# ROS node
rospy.init_node('ActionsFulfiller')
rospy.Subscriber(__communication.out_topic, __communication.out_message, __on_action_request)
rospy.spin()