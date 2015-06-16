#! /usr/bin/env python
#
#  Author: Francesco Trapani
#
# ROS node simulating simple contextual events.

import rospy
from contextCommunication import ContextCommunication

context = ContextCommunication()
topic = rospy.Publisher(context.in_topic, context.in_message, latch=True, queue_size=10)


def events():
    """
    Publishes on the input context topic some ContextInput messages representing simple contextual events.
    """
    rospy.sleep(1.5)

    assertions = {'doorClosed(5,4)': True, 'doorClosed(4,5)': True}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(2.0)

    assertions['doorClosed(5,4)'] = False
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)


def log(assertions):
    """
    Logs information about the published messages
    """
    log_message = "ContextInput - input sent:\n"
    for atom, value in assertions.iteritems():
        log_message += atom.__str__() + " = " + str(value) + "\n"
    rospy.loginfo(log_message)


# ROS node
if __name__ == '__main__':
    try:
        rospy.init_node('context_input')
        events()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
