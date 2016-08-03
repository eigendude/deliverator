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

import signal
import subprocess
import time

import rospy

DHCP_TIMEOUT_SEC = 1.0 # Bail if a lease is not obtained within this timeout
DHCLIENT_PATH    = '/sbin/dhclient'

LOG_DHCLIENT_ERROR = True

# For determining the username
try:
    import pwd
    import os
except ImportError:
    import getpass
    pwd = None

def currentUser():
    if pwd:
        return pwd.getpwuid(os.geteuid()).pw_name
    else:
        return getpass.getuser()

class DHCP:
    @classmethod
    def getLease(cls, interface):
        rospy.loginfo('Checking for IP address on [%s]' % interface)
        start = time.time()
        proc = subprocess.Popen(['sudo', '--non-interactive', DHCLIENT_PATH, '-1', interface])
        ret = None

        while ret is None:
            time.sleep(0.1)
            ret = proc.poll()
            if time.time() >= start + DHCP_TIMEOUT_SEC:
                break

        # If process is still running, kill it and return False
        if ret is None:
            #proc.send_signal(signal.SIGINT) # TODO: Can't kill sudo process
            return False

        if ret != 0:
            cls._logError(ret)
            return False

        return True

    @staticmethod
    def _logError(returnCode):
        global LOG_DHCLIENT_ERROR
        if LOG_DHCLIENT_ERROR:
            if returnCode == 1:
                # Probably a sudo permission error
                print('')
                print('***********************************************************')
                print('Error: This process requires sudo privileges for dhclient')
                print('Run "sudo visudo" and add the following line:')
                print('')
                print('%s ALL=(ALL) NOPASSWD: %s' % (currentUser(), DHCLIENT_PATH))
                print('***********************************************************')
                print('')
                LOG_DHCLIENT_ERROR = False
