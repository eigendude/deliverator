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

from DiagnosticsTypes import Battery

import os

POWER_SUPPLY_SYS_FS = '/sys/class/power_supply'

class BatteryDir:
    def __init__(self, dirName):
        self.path = os.path.join(POWER_SUPPLY_SYS_FS, dirName)

    def getCapacity(self):
        try:
            with open(os.path.join(self.path, 'capacity')) as f:
                capacity = int(f.read())
        except:
            capacity = -1.0
        return capacity

    def getVoltage(self):
        try:
            with open(os.path.join(self.path, 'voltage_now')) as f:
                voltage = int(f.read()) / 1000000.0
        except:
            voltage = -1.0
        return voltage

    def isDischarging(self):
        try:
            with open(os.path.join(self.path, 'status')) as f:
                status = f.read()
                bIsDischarging = status.startswith('Discharging')
        except:
            bIsDischarging = None
        return bIsDischarging

class BatterySysfs(Battery):
    def updateValues(self):
        batteries = self.getBatteries()
        self.percentage = sum([battery.getCapacity() for battery in batteries]) / float(len(batteries))
        self.voltage = sum([battery.getVoltage() for battery in batteries]) / float(len(batteries))
        self.discharging = all([battery.isDischarging() for battery in batteries])

    @staticmethod
    def getBatteries():
        batteries = [ ]

        power_sources = os.listdir(POWER_SUPPLY_SYS_FS)
        for source in power_sources:
            if source.startswith('BAT'):
                batteries.append(BatteryDir(source))

        return batteries

    @staticmethod
    def isSupported():
        return len(BatterySysfs.getBatteries()) > 0
