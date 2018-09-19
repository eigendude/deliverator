#!/usr/bin/env python
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

import rospy
from sensor_msgs.msg import Joy

BUTTON_A_ID = 0
BUTTON_B_ID = 1
BUTTON_X_ID = 2
BUTTON_Y_ID = 3
BUTTON_LEFT_SHOULDER_ID = 4
BUTTON_RIGHT_SHOULDER_ID = 5
BUTTON_SELECT_ID = 6
BUTTON_START_ID = 7
BUTTON_GUIDE_ID = 8
BUTTON_LEFT_STICK_ID = 9
BUTTON_RIGHT_STICK_ID = 10
BUTTON_LEFT_ID = 11
BUTTON_RIGHT_ID = 12
BUTTON_UP_ID = 13
BUTTON_DOWN_ID = 14

AXIS_RIGHT_ANALOG_X_ID = 3
AXIS_LEFT_TRIGGER_ID = 2
AXIS_RIGHT_TRIGGER_ID = 5

def xAxisValue(observed):
    return observed

def yAxisValue(observed):
    return observed

def triggerValue(observed):
    return (1 - observed) / 2

class Button:
    A = 'A'
    B = 'B'
    X = 'X'
    Y = 'Y'
    LEFT_SHOULDER = 'Left shoulder'
    RIGHT_SHOULDER = 'Right shoulder'
    SELECT = 'Select'
    START = 'Start'
    GUIDE = 'Guide'
    LEFT_STICK = 'Left stick'
    RIGHT_STICK = 'Right stick'
    LEFT = 'Left'
    RIGHT = 'Right'
    UP = 'Up'
    DOWN = 'Down'

class JoystickListener:

    def __init__(self):
        self.resetButtons()

    def resetButtons(self):
        self.buttons = {
            Button.A: 0,
            Button.B: 0,
            Button.X: 0,
            Button.Y: 0,
            Button.LEFT_SHOULDER: 0,
            Button.RIGHT_SHOULDER: 0,
            Button.SELECT: 0,
            Button.START: 0,
            Button.GUIDE: 0,
            Button.LEFT_STICK : 0,
            Button.RIGHT_STICK: 0,
            Button.LEFT: 0,
            Button.RIGHT: 0,
            Button.UP: 0,
            Button.DOWN: 0,
        }

    def __enter__(self):
        self.joySub = rospy.Subscriber('joy', Joy, self.joystickCallback)

    def __exit__(self, type, value, traceback):
        self.joySub.unregister()

    def registerCallback(self, callback, buttons=[ ], commands=True):
        self.callbackButtons = buttons
        self.callbackCommands = commands
        self.callback = callback

    def unregisterCallback(self):
        self.callback = None
        self.resetButtons()

    def joystickCallback(self, data):
        if self.callback:
            for button in self.callbackButtons:
                buttonIndex = self.getIndex(button)

                if not self.buttons[button] and data.buttons[buttonIndex]:
                    rospy.logdebug('Pressed %s', button)
                    self.callback.onPress(button)
                elif self.buttons[button] and not data.buttons[buttonIndex]:
                    rospy.logdebug('Released %s', button)
                    self.callback.onRelease(button)

                self.buttons[button] = data.buttons[buttonIndex]

            if self.callbackCommands:
                steering = xAxisValue(data.axes[AXIS_RIGHT_ANALOG_X_ID])

                leftTrigger = triggerValue(data.axes[AXIS_LEFT_TRIGGER_ID])
                rightTrigger = triggerValue(data.axes[AXIS_RIGHT_TRIGGER_ID])
                throttle = rightTrigger - leftTrigger

                self.callback.onCommand(steering, throttle)

    @staticmethod
    def getIndex(button):
        if button == Button.A:              return BUTTON_A_ID
        if button == Button.B:              return BUTTON_B_ID
        if button == Button.X:              return BUTTON_X_ID
        if button == Button.Y:              return BUTTON_Y_ID
        if button == Button.LEFT_SHOULDER:  return BUTTON_LEFT_SHOULDER_ID
        if button == Button.RIGHT_SHOULDER: return BUTTON_RIGHT_SHOULDER_ID
        if button == Button.SELECT:         return BUTTON_SELECT_ID
        if button == Button.START:          return BUTTON_START_ID
        if button == Button.GUIDE:          return BUTTON_GUIDE_ID
        if button == Button.LEFT_STICK:     return BUTTON_LEFT_STICK_ID
        if button == Button.RIGHT_STICK:    return BUTTON_RIGHT_STICK_ID
        if button == Button.LEFT:           return BUTTON_LEFT_ID
        if button == Button.RIGHT:          return BUTTON_RIGHT_ID
        if button == Button.UP:             return BUTTON_UP_ID
        if button == Button.DOWN:           return BUTTON_DOWN_ID
        raise Exception('Invalid button: %s' % button)
