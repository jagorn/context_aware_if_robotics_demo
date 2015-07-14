#!/usr/bin/env python
#
#  Author: Francesco Trapani
#
# ROS node wrapping the ContextMiddleware class
import sys
import rospy
import contextMiddleware


# ROS node
rospy.init_node('contextMiddleware', argv=sys.argv, anonymous=True)
semantic_files = rospy.myargv(sys.argv[1:])
reasoner = contextMiddleware.ContextMiddleware(semantic_files)
rospy.spin()
