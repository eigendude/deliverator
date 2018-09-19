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

from DHCP import DHCP
from Interface import InterfaceType
from InterfaceCallbacks import InterfaceCallbacks
from InterfaceScanner import InterfaceScanner
from Localization import Localization
from NetworkBridge import NetworkBridge
from NetworkCallbacks import NetworkCallbacks

import rospy

class NetworkServer(InterfaceCallbacks, NetworkCallbacks):
    def __init__(self):
        self._trustedNetwork = NetworkBridge()
        self._untrustedNetwork = NetworkBridge()
        self._localization = Localization()
        self._interfaceScanner = InterfaceScanner(self)

    def initialize(self):
        if not self._trustedNetwork.initialize() or \
           not self._untrustedNetwork.initialize() or \
           not self._localization.initialize():
            return False
        return True

    def deinitialize(self):
        self._localization.deinitialize()
        self._untrustedNetwork.deinitialize()
        self._trustedNetwork.deinitialize()

    def scanInterfaces(self):
        self._interfaceScanner.runScan()

    def interfaceAdded(self, iface):
        # Handle WiFi interfaces
        if iface.type() == InterfaceType.WIFI:
            rospy.loginfo('Registering wireless [%s]' % iface.name())
            self._localization.setInterface(iface)

        # Handle bridge interfaces
        elif iface.type() == InterfaceType.BRIDGE:
            rospy.loginfo('Registering bridge [%s]' % iface.name())
            if iface.isTrusted():
                self._trustedNetwork.addInterface(iface)
            else:
                self._untrustedNetwork.addInterface(iface)

        # Handle tap interfaces
        elif iface.type() == InterfaceType.TAP:
            rospy.loginfo('Registering tap [%s]' % iface.name())
            self._trustedNetwork.addInterface(iface)

        # Handle ethernet interfaces
        elif iface.type() == InterfaceType.ETHERNET:
            gateway = iface.getGateway()
            if not gateway:
                # Interface might not have IP address yet, try to get one now
                if DHCP.getLease(iface.name()):
                    gateway = iface.getGateway()

            if gateway:
                rospy.loginfo('Skipping interface [%s] with gateway %s' % (iface.name(), gateway))
            else:
                rospy.loginfo('Registering ethernet [%s]' % iface.name())
                self._untrustedNetwork.addInterface(iface)

    def interfaceRemoved(self, iface):
        rospy.loginfo('Unregistering [%s]' % iface.name())

        # Handle WiFi interfaces
        if iface.type() == InterfaceType.WIFI:
            self._localization.unsetInterface(iface)

        # Handle bridge interfaces
        elif iface.type() == InterfaceType.BRIDGE:
            if iface.isTrusted():
                self._trustedNetwork.removeInterface(iface)
            else:
                self._untrustedNetwork.removeInterface(iface)

        # Handle tap interfaces
        elif iface.type() == InterfaceType.TAP:
            self._trustedNetwork.removeInterface(iface)

        # Handle ethernet interfaces
        elif iface.type() == InterfaceType.ETHERNET:
            self._untrustedNetwork.removeInterface(iface)

    def enableAccessPoint(self):
        params = { 'name': 'Deliverator', 'channel': 11 } # TODO
        rospy.loginfo('Enabling wireless AP "%s" (channel %d)' % (params['name'], params['channel']))
        if self._localization.enableAccessPointMode(params):
            iface = self._localization.getInterface()
            iface.enableAccessPoint(self)
            self._trustedNetwork.addInterface(iface)

    def disableAccessPoint(self):
        rospy.loginfo('Disabling wireless AP')
        iface = self._localization.getInterface()
        self._trustedNetwork.removeInterface(iface)
        iface.disableAccessPoint()
        self._localization.disableAccesPointMode()

    def wifiClientConnected(self, client):
        self._localization.enableTargetMode(client)

    def wifiClientDisconnected(self, client):
        self._localization.disableTargetMode(client)

    def wifiConnect(self, params):
        rospy.loginfo('Connecting to WiFi network "%s" (channel %d)' % (params['name'], params['channel']))
        if self._localization.targetNetwork(params):
            self._untrustedNetwork.addInterface(self._localization.getInterface())

    def wifiDisconnect(self):
        rospy.loginfo('Disconnecting from WiFi network')
        self._untrustedNetwork.removeInterface(self._localization.getInterface())
        self._localization.untargetNetwork()
