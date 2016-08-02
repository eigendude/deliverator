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

from Interface import InterfaceType

class NetworkBridge:
    def __init__(self):
        self._bridge = None
        self._interfaces = []

    def initialize(self):
        return True

    def deinitialize(self):
        pass

    def addInterface(self, iface):
        if iface:
            if iface.type() == InterfaceType.BRIDGE and not self._bridge:
                self._bridge = iface

                # Bridge interfaces
                for interface in self._interfaces:
                    self._bridge.addInterface(interface)

            elif iface.name() not in [i.name() for i in self._interfaces]:
                self._interfaces.append(iface)

                # Bridge interface
                if self._bridge:
                    self._bridge.addInterface(iface)

    def removeInterface(self, iface):
        if iface:
            if iface.name() == self._bridge.name():
                # Remove interfaces
                for interface in self._interfaces:
                    self._bridge.removeInterface(interface)

                self._bridge = None

            elif iface.name() in [i.name() for i in self._interfaces]:
                # Remove interface
                if self._bridge:
                    self._bridge.removeInterface(iface)

                self._interfaces = [i for i in self._interfaces if i.name() != iface.name()]
