#!/usr/bin/env python
################################################################################
#
#      Copyright (C) 2016 juztamau5
#      Software License Agreement (MIT License)
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#      The above copyright notice and this permission notice shall be included
#      in all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#  THE SOFTWARE.
#
################################################################################

class Network(object):
    """
    @brief Abstraction of a computer network

    The network can be accessed by one or more local interfaces.
    """
    def __init__(self):
        self._interfaces = { }

    def initialize(self):
        """
        @brief Initialize the resources of the network
        @return True if the network was initialized successfully, false otherwise
        """
        return False

    def deinitialize(self):
        """
        @brief Deinitialize the resources of the network
        """
        pass

    def addInterface(self, iface):
        """
        @brief Add an interface through which a node can communicate

        @param iface    Interface - an interface through which two nodes can communicate
        """
        if iface and iface.name() not in self._interfaces:
            self._interfaces[iface.name()] = iface

    def removeInterface(self, iface):
        """
        @brief Remove an interface

        @param iface    Interface - the interface to remove
        """
        if iface and iface.name() in self._interfaces:
            self._interfaces.pop(iface.name())