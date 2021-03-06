#!/usr/bin/env python
################################################################################
#
#      Copyright (C) 2016-2018 juztamau5
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

import netifaces

class InterfaceType:
    UNKNOWN = 0
    WIFI = 1
    ETHERNET = 2
    BRIDGE = 3
    TAP = 4

class Interface(object):
    def __init__(self, name):
        self._name = name

    def name(self):
        return self._name

    def type(self):
        return InterfaceType.UNKNOWN

    def initialize(self):
        return True

    def deinitialize(self):
        pass

    def getAddress(self):
        addresses = netifaces.ifaddresses(self._name)

        if netifaces.AF_INET in addresses:
            for address in addresses[netifaces.AF_INET]:
                # Skip invalid addresses
                if address['addr'] == '0.0.0.0':
                    continue

                # Skip link-local addresses
                if address['addr'].startswith('169.254'):
                    continue

                return address['addr']

        return None

    def getGateway(self):
        gateways = netifaces.gateways()

        if netifaces.AF_INET in gateways:
            for gateway in gateways[netifaces.AF_INET]:
                interface = gateway[1]
                if (self._name == interface):
                    ipAddress = gateway[0]
                    return ipAddress

        return None
