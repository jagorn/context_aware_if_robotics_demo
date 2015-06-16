#!/usr/bin/env python
#
#  Authors: Benjamin Andres, Philipp Obermeier, Orkunt Sabuncu, Torsten Schaub, David Rajaratnam
#  Extended and modified by Francesco Trapani

import rospy
import actionlib
import gringo


class HandleCommunication:
    TopicName = None
    TopicType = None
    MessageToSolver = None
    Publisher = None
    GetTime = None
    AddSend = None
    AddReceive = None
    SetExternals = None
    SendMessages = []
    ReceivedMessages = []

    def __init__(self, publisher, topic_name, topic_type, message_to_solver):
        self.Publisher = publisher
        self.TopicName = topic_name
        self.TopicType = topic_type
        self.MessageToSolver = message_to_solver

    def start(self, get_time, add_send, add_receive, set_externals):
        self.GetTime = get_time
        self.AddSend = add_send
        self.AddReceive = add_receive
        self.SetExternals = set_externals
        rospy.Subscriber(self.TopicName, self.TopicType, self.receive_message)
        return True

    def send_message(self, message):
        if self.AddSend(message):
            self.Publisher.publish(message)

    def receive_message(self, message):
        if self.AddReceive(message):
            self.SetExternals(self.MessageToSolver(message, self.GetTime()))


class HandleRequest:

    GoalToSolver = None
    CancelToSolver = None
    GetTime = None
    SetExternals = None
    Server = None
    Goals = {}

    def __init__(self, action_name, action_type, goal_to_solver, cancel_to_solver):
        self.Server = actionlib.ActionServer(action_name, action_type, self.incoming, self.cancel, auto_start=False)
        self.GoalToSolver = goal_to_solver
        self.CancelToSolver = cancel_to_solver

    def start(self, get_time,set_externals):
        self.GetTime = get_time
        self.SetExternals = set_externals
        self.Server.start()

    def incoming(self, goal):

        # ROS log
        rospy.loginfo('ROSoClingo - request %s received\n', goal.goal.goal.request)

        self.Goals[len(self.Goals)+1] = goal
        self.SetExternals(self.GoalToSolver(self.Goals.keys()[self.Goals.values().index(goal)],
                                            goal.goal.goal.request, self.GetTime()))

    def cancel(self, goal):
        self.SetExternals(self.CancelToSolver(self.Goals.keys()[self.Goals.values().index(goal)], self.GetTime()))

    def handle_goal(self, id, command):
        if id in self.Goals:
            if command == "cancel":
                if self.Goals[id].get_goal_status().status in [self.Goals[id].get_goal_status().RECALLING,
                                                               self.Goals[id].get_goal_status().PREEMPTING]:
                    self.Goals[id].set_canceled()
            elif command == "success":
                if self.Goals[id].get_goal_status().status in [self.Goals[id].get_goal_status().ACTIVE,
                                                               self.Goals[id].get_goal_status().PREEMPTING]:
                    self.Goals[id].set_succeeded()
            elif command == "abort":
                if self.Goals[id].get_goal_status().status in [self.Goals[id].get_goal_status().ACTIVE,
                                                               self.Goals[id].get_goal_status().PREEMPTING]:
                    self.Goals[id].set_aborted()
            elif command == "accept":
                if self.Goals[id].get_goal_status().status in [self.Goals[id].get_goal_status().PENDING,
                                                               self.Goals[id].get_goal_status().RECALLING]:
                    self.Goals[id].set_accepted()
            elif command == "reject":
                if self.Goals[id].get_goal_status().status in [self.Goals[id].get_goal_status().PENDING,
                                                               self.Goals[id].get_goal_status().RECALLING]:
                    self.Goals[id].set_rejected()
            else:
                # ROS log
                rospy.logwarn("ROSoClingo - Warning: command: " + command + " in handleGoal not known!")
        else:
            # ROS log
            rospy.logwarn("ROSoClingo - Warning: Goal with id: " + str(id) + " does not exists! Command ignored!")


class HandleSolver:

    SolverToMessage = None
    Controller = None
    Solver = None
    Future = None
    Atoms = []
    Shown = []
    interrupted = None
    GetTime = None
    CheckTime = None
    HandleGoal = None
    SendMessage = None
    Step = 1

    def __init__(self, arguments, files, solver_to_message):
        self.SolverToMessage = solver_to_message
        self.Solver = gringo.Control(arguments)
        for file in files:
            self.Solver.load(file)
        self.Solver.ground([("base", [])])
        self.Solver.ground([("transition", [self.Step])])
        self.Solver.assign_external(gringo.Fun("horizon", [self.Step]), True)

    def start(self,get_time, check_time, handle_goal, send_message):
        self.GetTime = get_time
        self.CheckTime = check_time
        self.HandleGoal = handle_goal
        self.SendMessage = send_message
        self.Future = self.Solver.solve_async(None, self.on_model, self.on_finish)
        self.Future.wait()

    def on_model(self, model):
        self.Atoms[:] = model.atoms(gringo.Model.ATOMS)
        self.Shown[:] = model.atoms(gringo.Model.SHOWN)

    def on_finish(self, result, interrupted):
        self.interrupted = interrupted

    def set_externals(self, externals2values):
        self.Future.interrupt()

        while self.Step < self.GetTime():
            self.Step += 1
            if self.Step > 1:
                self.Solver.release_external(gringo.Fun("horizon", [self.Step-1]))
            self.Solver.ground([("transition", [self.Step])])
            self.Solver.assign_external(gringo.Fun("horizon", [self.Step]), True)

        for external, value in externals2values.iteritems():
            self.Solver.assign_external(external, value)

        self.interrupted = True
        self.Atoms = []
        self.Shown = []
        self.Future = self.Solver.solve_async([], self.on_model, self.on_finish)
        parts = []

        # ROS log
        log_message = "ROS0Clingo - started knowledge solving\n"
        rospy.loginfo(log_message)

        while self.Future.get() == gringo.SolveResult.UNSAT and self.interrupted is False:

            self.Step += 1
            parts.append(("transition", [self.Step]))
            if self.Step > 1:
                self.Solver.release_external(gringo.Fun("horizon", [self.Step-1]))
            self.Solver.ground(parts)
            self.Solver.assign_external(gringo.Fun("horizon", [self.Step]), True)

            self.interrupted = True
            self.Atoms = []
            parts = []
            self.Future = self.Solver.solve_async(None, self.on_model, self.on_finish)

        log_message = "ROS0Clingo - knowledge solved\n"
        if self.Future.get() == gringo.SolveResult.SAT and self.interrupted is False:
            self.CheckTime()

            # ROS log
            log_message += "plan elaborated:\n"
            for atom in self.Shown:
                log_message += atom.__str__() + "\n"
            log_message += "actions found:\n"

            for atom in self.Atoms:
                if atom.name() == "_request" and atom.args()[-1] == self.GetTime():
                    self.HandleGoal(atom.args()[0], "accept")
                elif atom.name() == "_reject" and atom.args()[-1] == self.GetTime():
                    self.HandleGoal(atom.args()[0], "reject")
                elif atom.name() == "_cancel" and atom.args()[-1] == self.GetTime():
                    self.HandleGoal(atom.args()[0], "cancel")
                elif atom.name() == "_successful" and atom.args()[-1] == self.GetTime():
                    self.HandleGoal(atom.args()[0], "success")
                elif atom.name() == "_impossible" and atom.args()[-1] == self.GetTime():
                    self.HandleGoal(atom.args()[0], "abort")
                elif atom.name() == "_action" and atom.args()[-1] == self.GetTime():

                    # ROS log
                    log_message += atom.__str__() + " at t = " + self.GetTime().__str__() + "\n"

                    self.Solver.ground([("action_commit", [atom.args()[0], atom.args()[1], atom.args()[2]])])
                    self.SendMessage(self.SolverToMessage(atom.args()[0], atom.args()[1]))
        rospy.loginfo(log_message)