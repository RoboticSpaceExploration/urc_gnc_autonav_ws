#!/usr/bin/env python3 
import rospy
import smach

from state.wait import Wait
from state.aruco_scan import ARUCOScan
from state.gate_traverse import GateTraverse
from state.spiral_search import SpiralSearch

def main():
    rospy.init_node('smach_scenario')

    sm = smach.StateMachine(outcomes=['detected', 'not_detected'])

    with sm:
        smach.StateMachine.add('SpiralSearch', SpiralSearch(),
                               transitions={'not_found':'not_detected',
                                            'found':'GateTraverse'}
                               )
        smach.StateMachine.add('GateTraverse', GateTraverse(),
                                transitions={'traversed':'detected',
                                                'failed':'not_detected'}
                                )
    outcome = sm.execute()
    rospy.spin()
    print(f"[***] SCENARIO CONCLUDED, OUTCOME {outcome}")

main()
