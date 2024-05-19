#!/usr/bin/env python3
import rospy
import smach
from gnc_executive.srv import StartMission, StartMissionResponse, CancelMission, CancelMissionResponse
from std_msgs.msg import String
from state.gate_traverse import GateTraverse
from state.goto_gnss import GoToGNSS
from state.spiral_search import SpiralSearch
from state.aruco_scan import ARUCOScan
from state.aruco_util import ARUCOUtil
from state.yolo_traverse import YOLOFollower
#from state.hex_search import HexagonalSearch
from state.repeat_gnss import SetRepeatLast

class MissionControl:
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])
        self.state_pub = rospy.Publisher('/mission_state', String, queue_size=10)
        self.goto_gnss_state = GoToGNSS()

        with self.sm:
            smach.StateMachine.add('GOTO_GNSS', self.goto_gnss_state,
                                   transitions={'succeeded_gnss': 'GOTO_GNSS',
                                                'succeeded_aruco': 'SEARCH_FOR_ARUCO_TAGS',
                                                'succeeded_yolo': 'SEARCH_FOR_YOLO',
                                                'succeeded': 'mission_completed',
                                                'failed': 'mission_failed'})
            smach.StateMachine.add('SEARCH_FOR_ARUCO_TAGS', SpiralSearch(),
                                   transitions={'found': 'GATE_TRAVERSE',
                                                'not_found': 'SET_REPEAT_LAST'})  # Changed transition on not_found to GOTO_GNSS
            smach.StateMachine.add('GATE_TRAVERSE', GateTraverse(),
                                   transitions={'traversed': 'GOTO_GNSS',
                                                'failed': 'SET_REPEAT_LAST'})
            smach.StateMachine.add('SEARCH_FOR_YOLO', SpiralSearch(),
                                     transitions={'found': 'YOLO_TRAVERSE',
                                                  'not_found': 'SET_REPEAT_LAST'})
            smach.StateMachine.add('YOLO_TRAVERSE', YOLOFollower(),
                                     transitions={'traversed': 'GOTO_GNSS',
                                                    'failed':'mission_failed'})

            smach.StateMachine.add('SET_REPEAT_LAST', SetRepeatLast(self.goto_gnss_state),
                                   transitions={'done': 'GOTO_GNSS'})

        # Attach a state change callback
        self.sm.register_transition_cb(self.state_transition_cb)

    def state_transition_cb(self, userdata, active_states):
        # Callback to publish the current state
        current_state = active_states[0] if active_states else 'none'
        self.state_pub.publish(current_state)

    def start_mission(self, request):
        outcome = self.sm.execute()
        rospy.loginfo(f"[***] SCENARIO CONCLUDED, OUTCOME {outcome}")
        return StartMissionResponse(success=True)

    def cancel_mission(self, request):
        self.sm.request_preempt()
        rospy.loginfo("Mission cancelled.")
        return CancelMissionResponse(success=True)

def main():
    rospy.init_node('smach_scenario')

    mission_control = MissionControl()
    
    s = rospy.Service('start_mission', StartMission, mission_control.start_mission)
    c = rospy.Service('cancel_mission', CancelMission, mission_control.cancel_mission)

    rospy.spin()

if __name__ == "__main__":
    main()

