#!/usr/bin/env python
#
#  Authors: Benjamin Andres, Philipp Obermeier, Orkunt Sabuncu, Torsten Schaub, David Rajaratnam
#  Extended and modified by Francesco Trapani
#
# Main ROS node of the reasoning module.
# This class initializes all the components of the reasoning module, and provides some utility functions
# to convert the messages received by the other components into ASP assertions

import rospy
import rospkg
import sys
import os
from gringo import Fun
import gringoParser
from rosoclingoCommunication import RosoclingoCommunication
import icontroller
import iroshandler
import iroscontext


def goal_to_solver(id, request, time):
    return {Fun("_request", [id, gringoParser.string2fun(request), time]): True}


def cancel_to_solver(id, time):
    return {Fun("_cancel", [id, time]): True}


def message_to_solver(message, time):
    return {Fun("_value", [gringoParser.string2fun(message.robot),
                           gringoParser.string2fun(message.action),
                           gringoParser.string2fun(message.value), time]): True}


def solver_to_message(robot, action):
    message = handler.out_message()
    message.robot = str(robot)
    message.action = str(action)
    return message


def context_to_solver(assertions2values):
    context = {}
    for assertion, value in assertions2values.iteritems():
        context_assertion = Fun("_context", [assertion])
        context[context_assertion] = value
    return context

# ROS node
rospy.init_node('ROSoClingo', argv=sys.argv, anonymous=True)

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('adaptive_controller')
semantic_files = rospy.myargv(sys.argv[1:])

robot_name = rospy.get_param("robot_name", "robot0")
handler = RosoclingoCommunication(robot_name)

publisher = rospy.Publisher(handler.out_topic, handler.out_message, latch=True, queue_size=10)
handle_communication = iroshandler.HandleCommunication(publisher, handler.in_topic, handler.in_message, message_to_solver)
handle_request = iroshandler.HandleRequest(handler.action_topic, handler.action, goal_to_solver, cancel_to_solver)
handle_solver = iroshandler.HandleSolver(semantic_files, solver_to_message)
handle_context = iroscontext.HandleContext(robot_name, context_to_solver)
rosoclingo = icontroller.ROSoClingo(handle_request,handle_solver, handle_communication, handle_context)
rospy.spin()
