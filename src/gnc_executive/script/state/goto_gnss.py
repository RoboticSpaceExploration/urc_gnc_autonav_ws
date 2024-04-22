#!/usr/bin/env python3
import rospy
import smach
from geographic_msgs.msg import GeoPointStamped  
from gnc_executive.srv import GetNextCoordinate
from move_base_msgs.msg import MoveBaseActionResult
from robot_localization.srv import FromLL, FromLLRequest

class GoToGNSS(smach.State):
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.gps_coordinate = 0 

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        #self.pub = rospy.Publisher("/gps_goal", GeoPointStamped, queue_size=10)
    
    def execute(self,userdata):

        rospy.loginfo("Starting")
        goal_gnss = self.get_next_coordinate_client()
        if goal_gnss is None:
            rospy.loginfo("Failed to get next coordinate.")
            return 'failed'

        rospy.wait_for_service('robot_localization/fromLL')
        gps2utm = rospy.ServiceProxy('robot_localization/fromLL', FromLL)

        request = FromLLRequest()
        request.ll_point.latitude = goal_gnss.position.latitude
        request.ll_point.longitude = goal_gnss.position.longitude
        
        response = gps2utm(request)

        rospy.loginfo(f"Navigating to {goal_gnss.position.latitude} {goal_gnss.position.longitude}...")

        goal = create_move_base_goal(x,y)

        client.send_goal(goal)
        client.wait_for_result() #Blocking, client.get_state() for result
       
        # TODO: succeed state
        return 'succeeded' 
    
    def create_move_base_goal(x,y):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "utm"

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.position.orientation.w = 1.0

        return goal

    @staticmethod
    def get_next_coordinate_client():
        rospy.wait_for_service('get_next_coordinate')
        try:
            get_next_coordinate = rospy.ServiceProxy('get_next_coordinate', GetNextCoordinate)
            resp = get_next_coordinate()
            return resp.next_coordinate
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None


