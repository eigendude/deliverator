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

from deliverator.controller import EmergencyStop
from deliverator.controller import FreeDrive
from deliverator.controller import Playback
from deliverator.controller import PlaybackReady
from deliverator.controller import ReturnToStart

import rospy
import smach
#import smach_ros

NODE_NAME = 'controller'

class DeliveratorController:
    def __init__(self):
        rospy.loginfo('Starting deliverator controller...')

        # Initialize the state machine
        self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        with self.sm:
            # Initial state is free drive mode
            smach.StateMachine.add('FREE_DRIVE',
                                   FreeDrive(),
                                   transitions={'playback': 'RETURN_TO_START', 'stop': 'EMERGENCY_STOP'})
            # Add mode for returning to start of a recording
            smach.StateMachine.add('RETURN_TO_START',
                                   ReturnToStart(),
                                   transitions={'ready': 'PLAYBACK_READY', 'stop': 'EMERGENCY_STOP'})
            # Add playback ready state
            smach.StateMachine.add('PLAYBACK_READY',
                                   PlaybackReady(),
                                   transitions={'start': 'PLAYBACK', 'stop': 'EMERGENCY_STOP'})
            # Add playback mode
            smach.StateMachine.add('PLAYBACK',
                                   Playback(),
                                   transitions={'stop': 'EMERGENCY_STOP'})
            # Add emergency stop mode
            smach.StateMachine.add('EMERGENCY_STOP',
                                   EmergencyStop(),
                                   transitions={'success': 'FREE_DRIVE', 'fatal': 'aborted'})

        # Set up the introspection server
        #self.sis = smach_ros.IntrospectionServer('deliverator_smach_server', self.sm, '/SM_ROOT')
        #self.sis.start()

    def Start(self):
        finalOutcome = self.sm.execute()

def main():
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    controller = DeliveratorController()
    controller.Start()

if __name__ == '__main__':
    main()
