import rospy
import smach

class GoToGNSS(smach.State):
    def __init__(self, waypoint):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.gps_coordinate = waypoint
        rospy.init_node("goto_gnss")
        self.pub = rospy.Publisher("/gps_goal")
        self.sub = rospy.Subscriber("/coordinates")

    def execute(self, userdata):
        rospy.loginfo(f"Navigating to {self.gps_coordinate}...")
       
        ## Read GPS coordinate / increment GPS counter

        ## Publish to gps2utm : StampedGeoPointTransform (publishes to move_base)
        pub.pub()
        

        return 'succeeded'

