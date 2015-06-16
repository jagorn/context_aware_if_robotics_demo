#!/usr/bin/env python
#
#  Authors: Benjamin Andres, Philipp Obermeier, Orkunt Sabuncu, Torsten Schaub, David Rajaratnam
#  Extended and modified by Francesco Trapani
#
# Controller of the reasoning module.
# This class starts all the components of the reasoning module (solver, communication, request and context
# handlers) and manages the time step cycles, on which the other components depend.

import rospy


class ROSoClingo:

    Time = 1

    Sent_messages = []
    Received_messages = []

    def __init__(self, handle_request, handle_solver, handle_communication, handle_context):
        """
        Initializes all the other components of the reasoning module (solver, communication, request and context
        handlers).
        """
        handle_solver.start(self.get_time, self.check_time, handle_request.handle_goal, handle_communication.send_message)
        handle_communication.start(self.get_time, self.add_send, self.add_receive, handle_solver.set_externals)
        handle_request.start(self.get_time, handle_solver.set_externals)
        handle_context.start(handle_solver.set_externals)

    def add_send(self, message):
        """
        Adds the outgoing action request to the sent messages pool.
        If an identical request is already in the pool, the message is ignored.
        If the pool already contains a request for the same robot,
        this old request is assumed to be obsolete and is substituted by the new one.
        """
        request_is_approved = True
        for sent in self.Sent_messages:
            if message.robot == sent.robot:

                # there is already an action queued for the same robot,
                # but a robot can perform an only action per cycle:
                if message.action == sent.action:

                    # the action already queued is a doublet of the new one:
                    # then just ignore the new one.
                    request_is_approved = False
                    break
                else:

                    # the action already queued is different from the new one:
                    # then the first action request is assumed to be obsolete,
                    # and only the new one is now considered
                    # TODO: must assure that the obsolete request is no longer fulfilled
                    self.Sent_messages.remove(sent)
                    rospy.logwarn("ROSoClingo - received request " + message.robot + ":" + message.action +
                                  " is in confict with previous request " + sent.robot + ":" + sent.action + ".\n" +
                                  "Obsolete request " + sent.robot + ":" + sent.action + " will be ignored.\n")
        if request_is_approved:
            self.Sent_messages.append(message)
        return request_is_approved

    def add_receive(self, message):
        """
        Adds the incoming action feedback to the received messages pool.
        If another feedback for the same robot is already in the pool, the message is ignored.
        If the feedback is relative to an unrequested action or to an obsolete request, the message is ignored.
        """
        feedback_is_consistent = False

        # check if the feedback is consistent respect to one of the active requests
        for sent in self.Sent_messages:
            if (message.robot == sent.robot) & (message.action == sent.action):
                # the received feedback is not relative to any of the active requests:
                # then just ignore it.
                feedback_is_consistent = True
                break

        if feedback_is_consistent:
            # check if the pool already contains a feedback for this robot
            for received in self.Received_messages:
                if message.robot == received.robot:
                    # a feedback for the same robot has already been received:
                    # then just ignore the newly received one.
                    feedback_is_consistent = False
                    break

        if feedback_is_consistent:
            self.Received_messages.append(message)
        return feedback_is_consistent

    def check_time(self):
        """
        Checks that all of the requested actions have received a feedback.
        If so, it ends the time cycle, resets all the message pools and starts a new time cycle.
        """
        is_completed = False

        # ROS log
        log_message = "ROSoClingo - action queue:"
        log_message += "\nsent: "
        for sent in self.Sent_messages:
            log_message += str(sent.robot) + ":" + str(sent.action) + " "
        log_message += "\nreceived: "
        for received in self.Received_messages:
            log_message += str(received.robot) + ":" + str(received.action) + " -> " + str(received.value)
        log_message += "\ntime = " + self.Time.__str__() + "\n"

        if self.Sent_messages != []:
            is_completed = True

            for sent in self.Sent_messages:
                satisfied_request = False
                for received in self.Received_messages:
                    if (received.robot == sent.robot) & (received.action == sent.action):
                        satisfied_request = True
                        break
                if not satisfied_request:
                    is_completed = False
                    break

        if is_completed:
            self.Time += 1
            self.Sent_messages = []
            self.Received_messages = []

            log_message += "cycle completed\n"

        rospy.loginfo(log_message)
        return is_completed

    def get_time(self):
        """
        Returns the value of the current time step.
        """
        return self.Time