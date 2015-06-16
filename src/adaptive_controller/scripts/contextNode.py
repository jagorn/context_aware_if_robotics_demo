#!/usr/bin/env python
#
#  Author: Francesco Trapani
#
# ROS node wrapping the ContextMiddleware class
import rospy
import contextMiddleware

# ROS node
rospy.init_node('contextMiddleware')
reasoner = contextMiddleware.ContextMiddleware()
rospy.spin()
