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

class Battery:
    def __init__(self):
        self.percentage = 0.0
        self.charging = True

class PowerSupply:
    def __init__(self):
        self.percentage = 0.0
        self.VIN = 0.0 # Volts
        self.V33 = 0.0 # Volts
        self.V5 = 0.0 # Volts
        self.V12 = 0.0 # Volts
        self.temperature = -1.0 # Celsius

class Temperature:
    def __init__(self):
        self.cpuTemp = -1.0 # Celsius
        self.fanSpeed = -1.0 # RPM

class Network:
    def __init__(self):
        self.wirelessLinkQuality = 0.0
        self.wirelessMaxQuality = 0.0
