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

import commands
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

NODE_NAME = 'manager'

class PowerStatus:
    ##  Initializes powerStatusMSG
    def __init__(self):
        self.valid = False
        self.percentage = 0

class PowerStatusACPI(PowerStatus):
    def getValues(self):
        outputString = commands.getoutput("acpi")
        if outputString == "No support for device type: power_supply":
            self.valid = False
        else:
            self.valid = True
        percentageLocation = outputString.find("%")
        if percentageLocation >= 0:
            self.percentage = int(outputString[percentageLocation - 3 : percentageLocation])

class PowerSupplyStatus:
    def __init__(self):
        self.valid = False
        self.percentage = 0
        self.VIN = 0
        self.V33 = 0
        self.V5  = 0
        self.V12 = 0
        self.temp = 0

    def getValues(self):
        pass

class TempStatus:
    def __init__(self):
        self.cpuTemp = -1.0
        self.fanSpeed = -1.0

class TempStatusLmSensors(TempStatus):
    def getValues(self):
        outputString = commands.getoutput("sensors | grep \"Core 0:\"")
        cpuTempLocation = outputString.find("+")
        if cpuTempLocation >= 0:
            try:
                self.cpuTemp1 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
            except ValueError:
                self.cpuTemp1 = -1.0
        else:
            self.cpuTemp1 = 0.0
        outputString = commands.getoutput("sensors | grep \"Core 1:\"")
        cpuTempLocation = outputString.find("+")
        if cpuTempLocation >= 0:
            try:
                self.cpuTemp2 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
            except ValueError:
                self.cpuTemp2 = -1.0
        else:
            self.cpuTemp2 = 0.0
        outputString = commands.getoutput("sensors | grep \"Core 2:\"")
        cpuTempLocation = outputString.find("+")
        if cpuTempLocation >= 0:
            try:
                self.cpuTemp3 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
            except ValueError:
                self.cpuTemp3 = -1.0
        else:
            self.cpuTemp3 = 0.0
        outputString = commands.getoutput("sensors | grep \"Core 3:\"")
        cpuTempLocation = outputString.find("+")
        if cpuTempLocation >= 0:
            try:
                self.cpuTemp4 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
            except ValueError:
                self.cpuTemp4 = -1.0
        else:
            self.cpuTemp4 = 0.0
        self.cpuTemp = max(self.cpuTemp1, self.cpuTemp2, self.cpuTemp3, self.cpuTemp4)
        outputString = commands.getoutput("sensors | grep \"fan2:\"")
        try:
            self.fanSpeed = float(outputString[23:27])
        except ValueError:
            self.fanSpeed = -1.0

class WirelessStatus:
    def __init__(self):
        self.valid = False
        self.linkQuality = 0
        self.maxQuality = 0

    def getValues(self):
        pass

if __name__ == '__main__':
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
    array = DiagnosticArray()
    status = DiagnosticStatus(name='SystemStatus', \
                              level=0, \
                              message='System Status')

    array.status = [status]

    powerStatusPublisher = PowerStatusACPI()
    powerSupplyHandler = PowerSupplyStatus()
    tempStatusPublisher = TempStatusLmSensors()
    wirelessStatusPublisher = WirelessStatus()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        status.values = []

        powerStatusPublisher.getValues()
        powerSupplyHandler.getValues()
        tempStatusPublisher.getValues()
        wirelessStatusPublisher.getValues()

        if powerStatusPublisher.valid:
            status.values.append(KeyValue(key='Laptop Battery Level', value=str(powerStatusPublisher.percentage)))

        if powerSupplyHandler.valid:
            status.values.append(KeyValue(key='Battery Level', value=str(powerSupplyHandler.percentage)))
            status.values.append(KeyValue(key='VIN', value=str(powerSupplyHandler.VIN)))
            status.values.append(KeyValue(key='3.3v Rail', value=str(powerSupplyHandler.V33)))
            status.values.append(KeyValue(key='5v Rail', value=str(powerSupplyHandler.V5)))
            status.values.append(KeyValue(key='12v Rail', value=str(powerSupplyHandler.V12)))
            status.values.append(KeyValue(key='Power Supply Temp', value=str(powerSupplyHandler.temp)))

        if tempStatusPublisher.cpuTemp != -1.0:
            status.values.append(KeyValue(key='CPU Temp', value=str(tempStatusPublisher.cpuTemp)))
        if tempStatusPublisher.fanSpeed != -1.0:
            status.values.append(KeyValue(key='Fan Speed', value=str(tempStatusPublisher.fanSpeed)))

        if wirelessStatusPublisher.valid:
            status.values.append(KeyValue(key='WiFi Quality', value=str(wirelessStatusPublisher.linkQuality)))
            status.values.append(KeyValue(key='WiFi Max Quality', value=str(wirelessStatusPublisher.maxQuality)))

        pub.publish(array)
        rate.sleep()
