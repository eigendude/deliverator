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

# Members are initialized with invalid values. Node must verify that detected
# values are valid (e.g. positive or not None).

class Battery(object):
    def __init__(self):
        self.percentage = -1.0
        self.voltage = -1.0 # Volts
        self.charging = None

class PowerSupply(object):
    def __init__(self):
        self.percentage = -1.0
        self.VIN = -1.0 # Volts
        self.V33 = -1.0 # Volts
        self.V5 = -1.0 # Volts
        self.V12 = -1.0 # Volts
        self.temperature = -1.0 # Celsius

class Temperature(object):
    def __init__(self):
        self.cpuTemp = -1.0 # Celsius
        self.fanSpeed = -1.0 # RPM

class Network(object):
    def __init__(self):
        self.wirelessLinkQuality = -1.0
        self.wirelessMaxQuality = -1.0
