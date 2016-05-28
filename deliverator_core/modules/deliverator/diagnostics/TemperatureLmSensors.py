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

from DiagnosticsTypes import Temperature

class TemperatureLmSensors(Temperature):
    def updateValues(self):
        cpuTemp1 = -1.0
        cpuTemp2 = -1.0
        cpuTemp3 = -1.0
        cpuTemp4 = -1.0

        outputString = commands.getoutput('sensors | grep "Core 0:"')
        cpuTempLocation = outputString.find('+')
        if cpuTempLocation >= 0:
            try:
                cpuTemp1 = float(outputString[cpuTempLocation + 1 : cpuTempLocation + 5])
            except ValueError:
                pass

        outputString = commands.getoutput('sensors | grep "Core 1:"')
        cpuTempLocation = outputString.find('+')
        if cpuTempLocation >= 0:
            try:
                cpuTemp2 = float(outputString[cpuTempLocation + 1 : cpuTempLocation + 5])
            except ValueError:
                pass

        outputString = commands.getoutput('sensors | grep "Core 2:"')
        cpuTempLocation = outputString.find('+')
        if cpuTempLocation >= 0:
            try:
                cpuTemp3 = float(outputString[cpuTempLocation + 1 : cpuTempLocation + 5])
            except ValueError:
                pass

        outputString = commands.getoutput('sensors | grep "Core 3:"')
        cpuTempLocation = outputString.find('+')
        if cpuTempLocation >= 0:
            try:
                cpuTemp4 = float(outputString[cpuTempLocation + 1 : cpuTempLocation + 5])
            except ValueError:
                pass

        self.cpuTemp = max(cpuTemp1, cpuTemp2, cpuTemp3, cpuTemp4)

        outputString = commands.getoutput('sensors | grep "fan2:"')
        try:
            self.fanSpeed = float(outputString[23 : 27])
        except ValueError:
            pass

    @staticmethod
    def isSupported():
        return True # TODO
