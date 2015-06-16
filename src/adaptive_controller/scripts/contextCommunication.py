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
    __in_message = ContextInput
    __out_message = ContextModel

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
            message.atoms.append(atom.__str__())
        return message

    def out_message2atoms(self, message):
        """
        Converts ContextOutput messages to ASP assertions
        """
        atoms = map(gringoParser.string2fun, message.atoms)
        return atoms