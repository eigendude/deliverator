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
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped

import threading

def clamp(minimum, value, maximum):
    return max(minimum, min(value, maximum))

class FreeDrive(smach.State):
    def __init__(self, joyListener):
        smach.State.__init__(self,
                             outcomes=['playback', 'stop'],
                             input_keys=['input'],
                             output_keys=['output'])

        self.joyListener = joyListener

        # Create an event to block on until we transition to the next state
        self.stateChangeEvent = threading.Event()

        self.car_ctl = AckermannDrive()
        self.car_msg = AckermannDriveStamped()
        self.cmdPub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped)

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
        steering = clamp(-1.0, steering, 1.0)
        throttle = clamp(-1.0, throttle, 1.0)

        #rospy.logdebug('Steering: %s', steering)
        #rospy.logdebug('  Throttle: %s', throttle)

        MAX_THROTTLE = 2 # m/s
        MAX_STEERING_ANGLE = 0.785398 # 45 deg

        self.car_ctl.speed = throttle * MAX_THROTTLE
        self.car_ctl.steering_angle = steering * MAX_STEERING_ANGLE
        self.car_msg.drive = self.car_ctl
        self.car_msg.header.stamp = rospy.Time.now()
        self.cmdPub.publish(self.car_msg)

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
