import smach
import rospy

class ARUCOScan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'not_found'])

    def execute(self, userdata):
        rospy.loginfo('Performing scan')
        
        

        return 'completed'

