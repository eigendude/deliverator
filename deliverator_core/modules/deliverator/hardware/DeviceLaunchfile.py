################################################################################
#
#      Copyright (C) 2017-2018 juztamau5
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

from Device import Device
from Process import Process

import rospkg
import rospy

import os
import subprocess

class DeviceLaunchfile(Device):
    def __init__(self, name, path , packageName, launchfile):
        super(DeviceLaunchfile, self).__init__(name, path)
        self._launchFile = self._getLaunchFile(packageName, launchfile)
        self._process = None

    def launch(self):
        # Make sure launch file exists
        if self._launchFile and os.path.exists(self._launchFile):
            try:
                popen = subprocess.Popen(['roslaunch',
                                           self._launchFile,
                                           'NAMESPACE:=' + rospy.get_namespace(),
                                           'DEVICE:=' + self._path])
                self._process = Process(popen)
                return True
            except:
                pass

        return False

    def kill(self):
        if self._process:
            self._process.kill()

    @staticmethod
    def _getLaunchFile(packageName, launchfile):
        packageDir = None

        rospack = rospkg.RosPack()
        try:
            packageDir = rospack.get_path(packageName)
        except rospkg.ResourceNotFound:
            pass
        else:
            return os.path.join(packageDir, 'launch', launchfile)

        return None
