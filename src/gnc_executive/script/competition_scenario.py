#!/usr/bin/env python3 
import rospy
import smach
from state.gate_traverse import GateTraverse
from state.goto_gnss import GoToGNSS
from state.spiral_search import SpiralSearch
from state.aruco_scan import ARUCOScan
from state.aruco_util import ARUCOUtil

def main():
    rospy.init_node('smach_scenario')

    sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])

    with sm:
        smach.StateMachine.add('GOTO_GNSS', GoToGNSS(waypoint=[0, 0]),
                               transitions={'succeeded':'SEARCH_FOR_ARUCO_TAGS',
                                            'failed':'mission_failed'})
        smach.StateMachine.add('SEARCH_FOR_ARUCO_TAGS', ARUCOScan(),
                               transitions={'completed':'GATE_TRAVERSE',
                                            'not_found':'SPIRAL_SEARCH'})
        smach.StateMachine.add('SPIRAL_SEARCH', SpiralSearch(),
                               transitions={'found':'SEARCH_FOR_ARUCO_TAGS',
                                            'not_found':'mission_failed'})
        smach.StateMachine.add('GATE_TRAVERSE', GateTraverse(),
                               transitions={'completed':'mission_completed',
                                            'failed': 'mission_failed'})

            
    outcome = sm.execute()
    rospy.spin()
    print(f"[***] SCENARIO CONCLUDED, OUTCOME {outcome}")

main()
