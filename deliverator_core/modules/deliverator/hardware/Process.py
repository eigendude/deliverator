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

import os
import signal
import time

TIMEOUT_SIGINT  = 5.0 #seconds
TIMEOUT_SIGTERM = 2.0 #seconds

class Process:
    def __init__(self, popen):
        self._popen = popen
        self._pgid = os.getpgid(popen.pid)

    def kill(self):
        if not self._popen:
            return # Already killed

        try:
            # Start with SIGINT and escalate from there
            timeout = time.time() + TIMEOUT_SIGINT
            os.killpg(self._pgid, signal.SIGINT)

            retcode = self._popen.poll()
            while time.time() < timeout and retcode is None:
                time.sleep(0.1)
                retcode = self._popen.poll()

            # Escalate non-responsive process
            if retcode is None:
                timeout = time.time() + TIMEOUT_SIGTERM
                os.killpg(self._pgid, signal.SIGTERM)

                retcode = self._popen.poll()
                while time.time() < timeout and retcode is None:
                    time.sleep(0.2)
                    retcode = self._popen.poll()

                if retcode is None:
                    os.killpg(self._pgid, signal.SIGKILL)

                    # Don't block on SIGKILL, because this results in more
                    # orphaned processes overall
                    #self._popen.wait()
                    #os.wait()

        finally:
            self._popen = None
