#! /usr/bin/env python
#
#  Author: Francesco Trapani
#
# ROS node observing and logging context model messages.
# Class used only for debugging purposes.

import rospy
from contextCommunication import ContextCommunication


def on_received_model(message):
    """
    Callback for ContextModel messages reception.
    Logs information about the model described in the received message.
    """
    log_message = "ContextOutput - output received:\n"
    for atom in message.atoms:
        log_message += atom.__str__() + "\n"
    rospy.loginfo(log_message)


# ROS node
if __name__ == '__main__':
    try:
        rospy.init_node('context_output')
        context = ContextCommunication()
        rospy.Subscriber(context.out_topic, context.out_message, on_received_model)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")