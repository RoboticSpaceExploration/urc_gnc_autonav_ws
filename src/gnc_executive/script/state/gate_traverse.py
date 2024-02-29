import rospy
import smach

class GateTraverse(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed','failed'])
        
    def execute(self, userdata):
        rospy.loginfo('Going through ARUCO tags')
        
        ## calculate goal points of ARUCO gate
        ## execute move_base
        ## reached goal?


        return 'completed'

