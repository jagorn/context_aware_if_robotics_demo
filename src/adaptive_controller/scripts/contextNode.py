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
robot_name = rospy.get_param("robot_name", "robot0")
reasoner = contextMiddleware.ContextMiddleware(robot_name, semantic_files)
rospy.spin()
