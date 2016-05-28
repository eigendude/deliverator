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

from DiagnosticsTypes import PowerSupply

import os
import subprocess

M4CTL_CMD = 'm4ctl'

class PowerSupplyM4API(PowerSupply):
    def updateValues(self):
        try:
            output = subprocess.check_output(M4CTL_CMD, shell=True)
        except:
            pass
        else:
            for line in output.split("\n"):
                tokens = line.split("\t")
                if len(tokens) is 2:
                    if tokens[0] == 'VIN:':
                        self.VIN = float(tokens[1])
                        # Battery max voltage 25, fully depleted 19, 6 volt span
                        self.percentage = min(100, ((float(tokens[1]) - 19) / 6.0) * 100)
                    if tokens[0] == '33V:':
                        self.V33 = float(tokens[1])
                    if tokens[0] == '5V:':
                        self.V5 = float(tokens[1])
                    if tokens[0] == '12V:':
                        self.V12 = float(tokens[1])
                    if tokens[0] == 'TEMP:':
                        self.temperature = float(tokens[1])

    @staticmethod
    def isSupported():
        # Must be root
        if os.geteuid() != 0:
            return False

        try:
            subprocess.check_output(M4CTL_CMD, shell=True)
        except:
            return False

        return True
