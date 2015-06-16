#!/usr/bin/env python
from adaptive_controller.msg import ActionFeedback, ActionRequest, TaskRequestAction, TaskRequestGoal


class RosoclingoCommunication:

    __action_topic = '/task_request'
    __in_topic = '/actions/feedback'
    __out_topic = '/actions/request'

    __action = TaskRequestAction
    __goal = TaskRequestGoal

    __in_message = ActionFeedback
    __out_message = ActionRequest

    @property
    def action(self):
        return self.__action

    @property
    def goal(self):
        return self.__goal

    @property
    def action_topic(self):
        return self.__action_topic

    @property
    def in_topic(self):
        return self.__in_topic

    @property
    def out_topic(self):
        return self.__out_topic

    @property
    def in_message(self):
        return self.__in_message

    @property
    def out_message(self):
        return self.__out_message