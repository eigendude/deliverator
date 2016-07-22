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

from Network import Network

class NetworkInternal(Network):
    """
    @brief Abstraction of a network that is running locally on the robot

    Several services are run on the local network, including DHCP and
    safety measures for untrusted networks.
    """
    def __init__(self, callbacks, trusted):
        """
        @param callbacks    NetworkCallbacks - callbacks invoked for network events
        @param trusted      bool - set to False to enable encryption and other network safety measures
        """
        super(Network, self).__init__()
        self._callbacks = callbacks
        self._trusted = trusted

    def isTrusted(self):
        return self._trusted

    def initialize(self):
        # TODO
        return True

    def deinitialize(self):
        # TODO
        pass

    def addInterface(self, iface):
        super(Network, self).addInterface(iface)
        # TODO: Bridge iface to network

    def removeInterface(self, iface):
        # TODO: Unbridge iface from network
        super(Network, self).removeInterface(iface)
