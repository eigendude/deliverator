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

class Localization:
    """
    @brief Provider of localization and tracking
    """
    def __init__(self):
        self._wifiInterface = None

    def initialize(self):
        return True

    def deinitialize(self):
        pass

    def setInterface(self, iface):
        """
        @brief Set the wireless adapter's local interface

        @param iface    InterfaceWiFi - the wireless adapter's interface
        """
        if iface and not self._wifiInterface:
            self._wifiInterface = iface
            self._wifiInterface.startPassiveScan()

    def unsetInterface(self, iface):
        """
        @brief Unset the wireless interface passed to setInterface()
        """
        if iface.name() == self._wifiInterface.name():
            self._wifiInterface.stopScan()
            self._wifiInterface = None

    def getInterface(self):
        """
        @brief Returns the wireless interface
        """
        return self._wifiInterface

    def enableAccessPointMode(self, params):
        pass # TODO

    def disableAccesPointMode(self):
        pass # TODO

    def enableTargetMode(self, client):
        pass # TODO

    def disableTargetMode(self, client):
        pass # TODO

    def targetNetwork(self, params):
        pass # TODO

    def untargetNetwork(self):
        pass # TODO
