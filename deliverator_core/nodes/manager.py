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

from deliverator.diagnostics import Diagnostics
from deliverator.hardware import Hardware

import rospy
import roslaunch
import rospkg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import os
import subprocess

NODE_NAME         = 'manager'
DIAGNOSTICS_TOPIC = '/diagnostics'
SCRIPT_DIR        = os.path.dirname(os.path.abspath(__file__))

def GetDiagnostics(diagnostics):
    values = []

    diagnostics.updateValues()

    if diagnostics.battery:
        if diagnostics.battery.percentage >= 0.0:
            values.append(KeyValue(key='Laptop Battery Level', value=str(diagnostics.battery.percentage)))
        if diagnostics.battery.voltage >= 0.0:
            values.append(KeyValue(key='Laptop Battery Voltage', value=str(diagnostics.battery.voltage)))
        if diagnostics.battery.charging is not None:
            values.append(KeyValue(key='Charging', value=str(diagnostics.battery.charging)))

    if diagnostics.powerSupply:
        if diagnostics.powerSupply.percentage >= 0.0:
            values.append(KeyValue(key='Battery Level', value=str(diagnostics.powerSupply.percentage)))
        if diagnostics.powerSupply.VIN >= 0.0:
            values.append(KeyValue(key='VIN', value=str(diagnostics.powerSupply.VIN)))
        if diagnostics.powerSupply.V33 >= 0.0:
            values.append(KeyValue(key='3.3v Rail', value=str(diagnostics.powerSupply.V33)))
        if diagnostics.powerSupply.V5 >= 0.0:
            values.append(KeyValue(key='5v Rail', value=str(diagnostics.powerSupply.V5)))
        if diagnostics.powerSupply.V12 >= 0.0:
            values.append(KeyValue(key='12v Rail', value=str(diagnostics.powerSupply.V12)))
        if diagnostics.powerSupply.temperature >= 0.0:
            values.append(KeyValue(key='Power Supply Temp', value=str(diagnostics.powerSupply.temperature)))

    if diagnostics.temperature:
        if diagnostics.temperature.cpuTemp >= 0.0:
            values.append(KeyValue(key='CPU Temp', value=str(diagnostics.temperature.cpuTemp)))
        if diagnostics.temperature.fanSpeed >= 0.0:
            values.append(KeyValue(key='Fan Speed', value=str(diagnostics.temperature.fanSpeed)))

    if diagnostics.network:
        if diagnostics.network.wirelessLinkQuality >= 0.0:
            values.append(KeyValue(key='WiFi Quality', value=str(diagnostics.network.wirelessLinkQuality)))
        if diagnostics.network.wirelessMaxQuality >= 0.0:
            values.append(KeyValue(key='WiFi Max Quality', value=str(diagnostics.network.wirelessMaxQuality)))

    return values

def GetLaunchDir():
    # Look for launch dir relative to this script
    launchDir = os.path.join(SCRIPT_DIR, '../launch')
    if os.path.exists(launchDir):
        return launchDir

    # Check if node is run from folder named after the package
    packageName = os.path.basename(SCRIPT_DIR)
    rospack = rospkg.RosPack()
    try:
        return os.path.join(rospack.get_path(packageName), 'launch')
    except rospkg.ResourceNotFound:
        pass

    return None

def LaunchHardwareNodes(hardware):
    launchDir = GetLaunchDir()
    if launchDir is not None:
        videoDevice = hardware.videoDevice
        if videoDevice is not None:
            launchfile = os.path.join(launchDir, 'camera.launch')
            subprocess.Popen(['roslaunch',
                              launchfile,
                              'NAMESPACE:=' + rospy.get_namespace(),
                              'DEVICE:=' + videoDevice])

def main():
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    pub = rospy.Publisher(DIAGNOSTICS_TOPIC, DiagnosticArray, queue_size=1)
    array = DiagnosticArray()
    status = DiagnosticStatus(name='SystemStatus', \
                              level=0, \
                              message='System Status')
    array.status = [status]

    diagnostics = Diagnostics()
    hardware = Hardware()

    LaunchHardwareNodes(hardware)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        status.values = GetDiagnostics(diagnostics)
        pub.publish(array)
        rate.sleep()

if __name__ == '__main__':
    main()
