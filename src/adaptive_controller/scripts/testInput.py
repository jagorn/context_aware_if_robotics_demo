#! /usr/bin/env python
#
#  Author: Francesco Trapani
#
# ROS node simulating simple contextual events.

import rospy
from contextCommunication import ContextCommunication
from adaptive_controller.msg import ActionRequest


def events():
    """
    Publishes on the input context topic some ContextInput messages representing simple contextual events.
    """

    assertions = {'light(dark)': True, '_time(night)': True}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(4.0)

    assertions = {'_person_at(nardi,27,3)': True, '_person_at(student,14,10)': True, '_person_at(student,21,10)': True}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(6.0)

    assertions = {'_time(morning)': True, '_light(dark)': False,'_time(night)': False}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(7.0)

    assertions = {'_person_at(student,19,3)': True}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(5.0)

    assertions = {'_person_at(student,23,2)': True}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(2.0)

    assertions = {'_person_at(student,27,5)': True, '_light(dark)': True}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(4.0)

    assertions = {'_time(morning)': False, '_time(noon)': True}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(3.0)

    assertions = { '_person_at(student,21,10)': False}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(6.0)

    assertions = { '_light(dark)': False}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(4.0)

    assertions = {'_person_at(student,14,10)': False, '_person_at(student,16,10)': True}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(5.0)

    assertions = {'_time(noon)': False, '_time(afternoon)': True}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)

    rospy.sleep(5.0)

    assertions = {'_person_at(student,27,3)': False, '_person_at(student,27,5)': False}
    msg = context.atoms_values2in_message(assertions)
    topic.publish(msg)
    log(assertions)


def log(assertions):
    """
    Logs information about the published messages
    """
    log_message = "CONTEXT EVENTS:\n\n"
    for atom, value in assertions.iteritems():
        log_message += atom.__str__() + " = " + str(value) + "\n"
    print  "\n\n\n\n" + log_message


# ROS node
if __name__ == '__main__':
    try:
        rospy.init_node('context_input')
        robot_name = rospy.get_param("robot_name", "robot0")

        context = ContextCommunication(robot_name)
        topic = rospy.Publisher(context.in_topic, context.in_message, latch=True, queue_size=10)
        tasks = rospy.Publisher("robot0/actions/request", ActionRequest, latch=True, queue_size=10)

        events()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
