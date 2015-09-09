#!/usr/bin/env python
#
#  Author: Francesco Trapani
#
# Class for the estimation of the contextual model.
# This class handles an ASP reasoner, which takes in input context assertions and outputs a contextual model.
# When initialized, the static knowledge stored in an lp file is loaded.
# Every time an input message is received, the new assertions are added to the knowledge, and a new model is estimated.

import rospy
import rospkg
import gringo
import contextCommunication

class ContextMiddleware:

    __context_inferences_source = '/asp/context.lp'
    __communication = None
    __model_publisher = None
    __roi_publisher = None
    __solver = None
    __future = None
    __interrupted = None
    __model = []

    def __init__(self, robot_name, semantic_files):
        """
        Class inizialization.
        The solver is initialized and the static knowledge is loaded from a file.
        """
        self.__communication = contextCommunication.ContextCommunication(robot_name)

        package = rospkg.RosPack()
        context_path = package.get_path('adaptive_controller') + self.__context_inferences_source

        self.__solver = gringo.Control()
        self.__solver.load(context_path)
        for semantic_file in semantic_files:
            self.__solver.load(semantic_file)
        self.__solver.ground([("base", [])])

        self.__future = self.__solver.solve_async(None, self.__on_model, self.__on_finish)
        self.__future.wait()

        self.__model_publisher = rospy.Publisher(self.__communication.out_topic, self.__communication.out_message, latch=True, queue_size=10)
        self.__roi_publisher = rospy.Publisher(self.__communication.roi_topic, self.__communication.roi_message, latch=True, queue_size=10)

        rospy.Subscriber(self.__communication.in_topic, self.__communication.in_message, self.__on_received_context)

    def __on_received_context(self, input_msg):
        """
        Callback for ContextInput messages reception.
        The assertions contained in the incoming messages are loaded, and a  new model is estimated
        """
        log_message = "ContextMiddleware - input received:\n"
        self.__future.interrupt()
        atoms2values = self.__communication.in_message2atoms_values(input_msg)
        for atom, value in atoms2values.iteritems():
            self.__solver.assign_external(atom, value)
            log_message += str(atom) + " = " + str(value) + "\n"
        rospy.loginfo(log_message)

        self.__interrupted = True
        self.__model = []
        self.__future = self.__solver.solve_async([], self.__on_model, self.__on_finish)
        if self.__future.get() == gringo.SolveResult.SAT and self.__interrupted is False:
            self.__publish_context()

    def __on_model(self, model):
        self.__model[:] = model.atoms(gringo.Model.SHOWN)

    def __on_finish(self, result, interrupted):
        self.__interrupted = interrupted

    def __publish_context(self):
        """
        Publishes a message with all the assertions contained in the new current context model
        """
        model_msg = self.__communication.atoms2out_message(self.__model)
        roi_msg = self.__communication.atoms2roi_message(self.__model)

        self.__model_publisher.publish(model_msg)
        self.__roi_publisher.publish(roi_msg)

        # ROS log
        log_message = "ContextMiddleWare - current model:\n"
        for atom in self.__model:
            log_message += atom.__str__() + "\n"
        rospy.loginfo(log_message)