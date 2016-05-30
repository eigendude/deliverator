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

import rospy
import smach

class ReturnToStart(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['ready', 'stop'],
                             input_keys=['input'],
                             output_keys=['output'])

    def execute(self, userdata):
        #i = 0
        #while (i < 10):
        #    if (self.preempt_requested()):
        #        rospy.logwarn('ReturnToStart Preempted!')
        #        return 'stop'
        #    print i
        #    i += 1
        #    rospy.sleep(5.0)
        rospy.sleep(3.0)
        output = { }
        output['error'] = 'None'
        output['data'] = None
        userdata.output = output
        return 'stop'
