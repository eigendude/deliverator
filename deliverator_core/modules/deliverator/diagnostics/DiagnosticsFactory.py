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

from BatterySysfs import BatterySysfs
from NetworkIwconfig import NetworkIwconfig
from PowerSupplyM4API import PowerSupplyM4API
from TemperatureLmSensors import TemperatureLmSensors

class DiagnosticsFactory:
    @staticmethod
    def createBattery():
        if BatterySysfs.isSupported():
            return BatterySysfs()
        return None

    @staticmethod
    def createPowerSupply():
        if PowerSupplyM4API.isSupported():
            return PowerSupplyM4API()
        return None

    @staticmethod
    def createTemperature():
        if TemperatureLmSensors.isSupported():
            return TemperatureLmSensors()
        return None

    @staticmethod
    def createNetwork():
        if NetworkIwconfig.isSupported():
            return NetworkIwconfig()
        return None
