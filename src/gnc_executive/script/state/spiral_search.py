import smach
import rospy

class SpiralSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found','not_found'])

    def execute(self, userdata):
        rospy.loginfo('Performing spiral search')
        
        ## Start spiral search node
        ## Wait for ARUCO callback
        ## Spiral search complete or aruco tag = 2

        return 'completed'

