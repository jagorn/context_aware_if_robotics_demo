#!/usr/bin/env python
#
#  Author: Francesco Trapani
#
# Facade for context comunication.
# Provides topics and messages for publishing input contextual information and subscribing to context estimates
# Provides methods for converting context messages to ASP assertions and vice versa.
# Each node subscribing/publishing context messages should do so using the topics/messages provided by this class.

import gringoParser
from adaptive_controller.msg import *


class ContextCommunication:

    __in_topic = '/context/input'
    __out_topic = '/context/model'
    __roi_topic = '/context/roi'

    __in_message = ContextInput
    __out_message = ContextModel
    __roi_message = ContextROIs

    __roi_predicate = 'context_area'

    @property
    def in_topic(self):
        return self.__in_topic

    @property
    def out_topic(self):
        return self.__out_topic

    @property
    def roi_topic(self):
        return self.__roi_topic

    @property
    def in_message(self):
        return self.__in_message

    @property
    def out_message(self):
        return self.__out_message

    @property
    def  roi_message(self):
        return self.__roi_message


    def atoms_values2in_message(self, atoms2values):
        """
        Converts ASP assertions (with their respective truth value) to ContextInput messages
        """
        message = self.__in_message()
        message.atoms = []
        message.values = []
        for atom, value in atoms2values.iteritems():
            message.atoms.append(atom.__str__())
            message.values.append(value)
        return message

    def in_message2atoms_values(self, message):
        """
        Converts ContextInput messages to ASP assertions (with their respective truth value)
        """
        atoms = map(gringoParser.string2fun, message.atoms)
        values = message.values
        atoms2values = {}
        for atom, value in zip(atoms, values):
            atoms2values[atom] = value
        return atoms2values

    def atoms2out_message(self, atoms):
        """
        Converts ASP assertions to ContextOutput assertions
        """
        message = self.__out_message()
        message.atoms = []
        for atom in atoms:
            if atom.name() != self.__roi_predicate:
                message.atoms.append(atom.__str__())
        return message

    def atoms2roi_message(self, atoms):
        """
        Converts ASP assertions to ContextROIs messages
        """
        message = self.__roi_message()
        message.ROIs = []
        for atom in atoms:
            if atom.name() == self.__roi_predicate:
                roi = ContextROI()
                args = atom.args()

                roi.id = args[0].__str__()
                roi.shape = args[1].__str__()
                roi.scaled = int(args[2].__str__())
                roi.centreX = int(args[3].__str__())
                roi.centreY = int(args[4].__str__())
                roi.radius = int(args[5].__str__())
                roi.side = int(args[6].__str__())
                roi.height = int(args[7].__str__())
                roi.weight = int(args[8].__str__())

                message.ROIs.append(roi)
        return message

    def out_message2atoms(self, message):
        """
        Converts ContextOutput messages to ASP assertions
        """
        atoms = map(gringoParser.string2fun, message.atoms)
        return atoms