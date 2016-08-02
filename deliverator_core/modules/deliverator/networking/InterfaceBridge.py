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

from Interface import Interface
from Interface import InterfaceType

import rospy

# Underscore in interface name breaks UFW firewall, so use a dash
BRIDGE_TRUSTED = 'br-trusted'
BRIDGE_UNTRUSTED = 'br-untrusted'

class InterfaceBridge(Interface):
    def __init__(self, trusted):
        super(InterfaceBridge, self).__init__(BRIDGE_TRUSTED if trusted else BRIDGE_UNTRUSTED)
        self._trusted = trusted

    def type(self):
        return InterfaceType.BRIDGE

    def isTrusted(self):
        return self._trusted

    @staticmethod
    def checkIsTrusted(name):
        return name == BRIDGE_TRUSTED

    @staticmethod
    def checkIsUntrusted(name):
        return name == BRIDGE_UNTRUSTED

    def addInterface(self, interface):
        rospy.loginfo('Adding interface %s to bridge %s', (interface.name(), self.name()))
        # TODO

    def removeInterface(self, interface):
        rospy.loginfo('Removing interface %s from bridge %s', (interface.name(), self.name()))
        # TODO
