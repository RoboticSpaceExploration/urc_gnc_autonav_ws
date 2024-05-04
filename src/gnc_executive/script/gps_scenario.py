#!/usr/bin/env python3 
import rospy
import smach
from state.goto_gnss import GoToGNSS
def main():
    rospy.init_node('smach_scenario')

    rospy.loginfo("Starting GPS waypoint State Machine")
    sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])

    with sm:
        
        smach.StateMachine.add('GOTO_GNSS', GoToGNSS(),
                               transitions={'succeeded':'GOTO_GNSS',
                                            'failed':'mission_failed'})


    outcome = sm.execute()
    rospy.spin()
    print(f"[***] SCENARIO CONCLUDED, OUTCOME {outcome}")

main()
