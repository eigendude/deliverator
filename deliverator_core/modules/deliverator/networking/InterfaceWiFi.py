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

import rospy
from deliverator_msgs.srv import CheckIsWireless
from deliverator_msgs.srv import EndScan
from deliverator_msgs.srv import StartScan

WIFI_SERVICE       = 'check_is_wireless'
START_SCAN_SERVICE = 'start_wifi_scan'
END_SCAN_SERVICE   = 'end_wifi_scan'

class InterfaceWiFi(Interface):
    def __init__(self, name):
        super(InterfaceWiFi, self).__init(name)

    def isWireless(self):
        return True

    @staticmethod
    def checkIsWireless(interfaceName):
        result = False

        rospy.wait_for_service(WIFI_SERVICE)

        serviceProxy = rospy.ServiceProxy(WIFI_SERVICE, CheckIsWireless)

        try:
            result = serviceProxy(interfaceName)
        except rospy.ServiceException as ex:
            rospy.logerror('Service did not process request: ' + str(ex))

        return result

    def startPassiveScan(self):
        result = False

        rospy.wait_for_service(START_SCAN_SERVICE)

        serviceProxy = rospy.ServiceProxy(START_SCAN_SERVICE, StartScan)

        try:
            serviceProxy(self.name(), True, [], [])
            result = True
        except rospy.ServiceException as ex:
            rospy.logerror('Service did not process request: ' + str(ex))

        return result

    def endScan(self):
        rospy.wait_for_service(END_SCAN_SERVICE)

        serviceProxy = rospy.ServiceProxy(END_SCAN_SERVICE, EndScan)

        try:
            serviceProxy(self.name())
        except rospy.ServiceException as ex:
            rospy.logerror('Service did not process request: ' + str(ex))
