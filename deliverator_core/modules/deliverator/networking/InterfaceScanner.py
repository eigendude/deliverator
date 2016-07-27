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

from InterfaceBridge import InterfaceBridge
from InterfaceEthernet import InterfaceEthernet
from InterfaceTap import InterfaceTap
from InterfaceWireless import InterfaceWireless

import netifaces

# Underscore in interface name breaks UFW, so use a dash
BRIDGE_NAME_TRUSTED = 'br-trusted'
BRIDGE_NAME_UNTRUSTED = 'br-untrusted'

# Prefix for tap devices, e.g. 'tap' for tap0
TAP_NAME_PREFIX = 'tap'

class InterfaceScanner:
    def __init__(self, callbacks):
        self._callbacks = callbacks
        self._interfaces = { }

    def runScan(self):
        scanResults = netifaces.interfaces()

        # Check for removed interfaces
        interfaces = self._interfaces[:]
        for ifaceName in interfaces:
            if ifaceName not in scanResults:
                self._callbacks.interfaceRemoved(interfaces[ifaceName])
                self._interfaces.pop(ifaceName)

        # Check for added interfaces
        for ifaceName in scanResults:
            # Skip loopback interface
            if ifaceName == 'lo':
                continue

            # Skip tap device
            if ifaceName.startswith('TAP_NAME_PREFIX'):
                continue 

            # Handle bridges
            if ifaceName in [BRIDGE_NAME_TRUSTED, BRIDGE_NAME_UNTRUSTED]:
                iface = InterfaceBridge(ifaceName)
            else:
                isWireless = False # TODO
                if isWireless:
                    iface = InterfaceWireless(ifaceName)
                else:
                    iface = InterfaceEthernet(ifaceName)

            if iface:
                self._interfaces[iface.name()] = iface
                self._callbacks.interfaceAdded(iface)
