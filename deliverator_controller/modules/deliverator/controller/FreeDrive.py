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

from JoystickListener import Button

import rospy
import smach
from sensor_msgs.msg import Joy

import threading

class FreeDrive(smach.State):
    def __init__(self, joyListener):
        smach.State.__init__(self,
                             outcomes=['playback', 'stop'],
                             input_keys=['input'],
                             output_keys=['output'])

        self.joyListener = joyListener

        # Create an event to block on until we transition to the next state
        self.stateChangeEvent = threading.Event()

    def onPress(self, button):
        if button == Button.A:
            self.beginRecording()
        elif button == Button.B:
            self.emergencyStop()
        elif button == Button.X:
            self.playback()

    def onRelease(self, button):
        pass

    def onCommand(self, steering, throttle):
        #rospy.logdebug('Steering: %s', steering)
        #rospy.logdebug('  Throttle: %s', throttle)
        pass # TODO

    def beginRecording(self):
        if not self.recording:
            self.recording = True
            rospy.logdebug('Beginning recording (TODO)')
            # TODO

    def endRecording(self):
        if self.recording:
            self.recording = False
            rospy.logdebug('Ending recording (TODO)')
            # TODO

    def emergencyStop(self):
        self.nextState = 'stop'
        self.stateChangeEvent.set()

    def playback(self):
        self.nextState = 'playback'
        self.stateChangeEvent.set()

    def execute(self, userdata):
        # Transition to E-stop by default
        self.nextState = 'stop'

        # Initialize recording state
        self.recording = False

        self.joyListener.registerCallback(self,
                                          buttons=[Button.A,
                                                   Button.B,
                                                   Button.X],
                                          commands=True)

        # Block until state changes or ROS is shutdown
        while not rospy.is_shutdown():
            if self.stateChangeEvent.wait(1):
                break
        self.stateChangeEvent.clear()

        self.joyListener.unregisterCallback()

        # Reset recording state
        self.endRecording()

        output = { }
        output['error'] = 'None'
        output['data'] = None
        userdata.output = output
        return self.nextState
