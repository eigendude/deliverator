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

from InterfaceEthernet import InterfaceEthernet
from InterfaceWiFi import InterfaceWiFi

import netifaces

# Underscore in interface name breaks UFW, so use a dash
BRIDGE_NAME_TRUSTED = 'br-trusted'
BRIDGE_NAME_UNTRUSTED = 'br-untrusted'

class InterfaceScanner:
    def __init__(self, callbacks):
        self._callbacks = callbacks
        self._interfaces = { }

    def runScan(self):
        interfaces = netifaces.interfaces()

        # Check for removed interfaces
        oldInterfaces = self._interfaces[:]
        for iface in oldInterfaces:
            if iface.name() in interfaces:
                continue
            self._callbacks.interfaceRemoved(iface)
            self._interfaces.pop(iface.name())

        # Check for added interfaces
        for iface in interfaces:
            if iface in ['lo', BRIDGE_NAME_TRUSTED, BRIDGE_NAME_UNTRUSTED]:
                continue
            if iface in self._interfaces:
                continue
            newInterface = self._createInterface(iface)
            self._interfaces[iface] = newInterface
            self._callbacks.interfaceAdded(newInterface)

    @staticmethod
    def _createInterface(name):
        addresses = netifaces.ifaddresses(name)

        hasIP = False
        if netifaces.AF_INET in addresses:
            for address in addresses[netifaces.AF_INET]:
                if address['addr'] == '0.0.0.0':
                    continue
                if address['addr'].startswith('169.254'):
                    continue
                hasIP = True
                break

        # TODO: Check for WiFi
        return InterfaceEthernet(name, hasIP)
