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

from DeviceJoystick import DeviceJoystick
from DeviceV4L import DeviceV4L
from DeviceWiFi import DeviceWiFi

import os

class HardwareScanner:
    @staticmethod
    def getDevices():
        devices = [ ]

        # Scan for V4L devices
        for deviceName in os.listdir('/dev'):
            if deviceName.startswith('video'):
                devicePath = os.path.join('/dev', deviceName)
                devices.append(DeviceV4L(deviceName, devicePath))
                break

        # Scan for joysticks
        for deviceName in os.listdir('/dev/input'):
            if deviceName.startswith('js0'): # TODO: detect 360 controllers attached to receiver
                devicePath = os.path.join('/dev/input', deviceName)
                devices.append(DeviceJoystick(deviceName, devicePath))
                break

        # TODO: Scan for WiFI devices

        return devices
