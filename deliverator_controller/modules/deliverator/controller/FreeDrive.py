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
from sensor_msgs.msg import Joy

import threading

BUTTON_A_ID = 0
BUTTON_B_ID = 1
BUTTON_X_ID = 2

AXIS_RIGHT_ANALOG_X_ID = 3
AXIS_LEFT_TRIGGER_ID = 2
AXIS_RIGHT_TRIGGER_ID = 5

def xAxisValue(observed):
    return -observed

def yAxisValue(observed):
    return observed

def triggerValue(observed):
    return (1 - observed) / 2

class FreeDrive(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['playback', 'stop'],
                             input_keys=['input'],
                             output_keys=['output'])

        # Initialize button states
        self.joystickButtons = {
            'A': 0,
            'B': 0,
            'X': 0,
        }

        # Initialize recording state
        self.recording = False

        # Transition to E-stop by default
        self.nextState = 'stop'

        # Create an event to block on until we transition to the next state
        self.stateChangeEvent = threading.Event()

        # Install shutdown hook
        #rospy.on_shutdown(self.emergencyStop)

    def joystickCallback(self, data):
        # Process buttons
        buttons = {
            'A': data.buttons[BUTTON_A_ID],
            'B': data.buttons[BUTTON_B_ID],
            'X': data.buttons[BUTTON_X_ID],
        }
        for button in self.joystickButtons:
            if not self.joystickButtons[button] and buttons[button]:
                self.onPress(button)
            elif self.joystickButtons[button] and not buttons[button]:
                self.onRelease(button)
        self.joystickButtons = buttons

        # Process axes
        steering = xAxisValue(data.axes[AXIS_RIGHT_ANALOG_X_ID])
        leftTrigger = triggerValue(data.axes[AXIS_LEFT_TRIGGER_ID])
        rightTrigger = triggerValue(data.axes[AXIS_RIGHT_TRIGGER_ID])
        throttle = rightTrigger - leftTrigger
        self.processAxes(steering, throttle)

    def onPress(self, button):
        rospy.logdebug('Pressed: %s', button)
        if button == 'A':
            self.beginRecording()
        elif button == 'B':
            self.emergencyStop()
        elif button == 'X':
            self.playback()

    def onRelease(self, button):
        rospy.logdebug('Released: %s', button)

    def processAxes(self, steering, throttle):
        #rospy.logdebug('Steering: %s', steering)
        #rospy.logdebug('  Throttle: %s', throttle)
        # TODO
        pass

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

    def request_preempt(self):
        rospy.logdebug('Here2')
        self.emergencyStop()

    def execute(self, userdata):
        recording = False

        joySub = rospy.Subscriber('joy', Joy, self.joystickCallback)

        # Block until state changes or ROS is shutdown
        while not rospy.is_shutdown():
            if self.stateChangeEvent.wait(1):
                break
        self.stateChangeEvent.clear()

        joySub.unregister()

        # Reset buttons
        for button in self.joystickButtons:
            if self.joystickButtons[button]:
                self.onRelease(button)
                self.joystickButtons[button] = 0

        # Reset recording state
        self.endRecording()

        output = { }
        output['error'] = 'None'
        output['data'] = None
        userdata.output = output
        return self.nextState
