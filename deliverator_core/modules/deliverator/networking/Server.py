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

from InterfaceCallbacks import InterfaceCallbacks
from InterfaceScanner import InterfaceScanner
from Localization import Localization
from NetworkCallbacks import NetworkCallbacks
from NetworkExternal import NetworkExternal
from NetworkInternal import NetworkInternal

import rospy

class Server(InterfaceCallbacks, NetworkCallbacks):
    def __init__(self, trusted):
        self._trusted = trusted
        self._localNetwork = NetworkInternal(self, trusted)
        self._externalNetwork = NetworkExternal()
        self._localization = Localization()

    def initialize(self):
        if not self._localNetwork.initialize() or \
           not self._externalNetwork.initialize() or \
           not self._localization.initialize():
            return False

        self._interfaceScanner = InterfaceScanner(self)
        self._interfaceScanner.runScan()

        return True

    def deinitialize(self):
        self._localization.deinitialize()
        self._externalNetwork.deinitialize()
        self._localNetwork.deinitialize()

    def interfaceAdded(self, iface):
        if iface.isWireless():
            rospy.loginfo('Registering wireless interface %s' % iface.name())
            self._localization.setInterface(iface)
        elif iface.hasGateway():
            rospy.loginfo('Registering external interface %s' % iface.name())
            self._externalNetwork.addInterface(iface)
        else:
            rospy.loginfo('Registering wired interface %s' % iface.name())
            self._localNetwork.addInterface(iface)

    def interfaceRemoved(self, iface):
        rospy.loginfo('Unregistering interface %s' % iface.name())
        self._externalNetwork.removeInterface(iface)
        self._localNetwork.removeInterface(iface)
        self._localization.unsetInterface(iface)

    def enableAccessPoint(self):
        params = { 'name': 'Deliverator', 'channel': 11 } # TODO
        rospy.loginfo('Enabling wireless AP "%s" (channel %d)' % (params['name'], params['channel']))
        if self._localization.enableAccessPointMode(params):
            iface = self._localization.getInterface()
            iface.enableAccessPoint(self)
            self._localNetwork.addInterface(iface)

    def disableAccessPoint(self):
        rospy.loginfo('Disabling wireless AP')
        iface = self._localization.getInterface()
        self._localNetwork.removeInterface(iface)
        iface.disableAccessPoint()
        self._localization.disableAccesPointMode()

    def wifiClientConnected(self, client):
        self._localization.enableTargetMode(client)

    def wifiClientDisconnected(self, client):
        self._localization.disableTargetMode(client)

    def wifiConnect(self, params):
        rospy.loginfo('Connecting to WiFi network "%s" (channel %d)' % (params['name'], params['channel']))
        if self._localization.targetNetwork(params):
            self._externalNetwork.addInterface(self._localization.getInterface())

    def wifiDisconnect(self):
        rospy.loginfo('Disconnecting from WiFi network')
        self._externalNetwork.removeInterface(self._localization.getInterface())
        self._localization.untargetNetwork()
