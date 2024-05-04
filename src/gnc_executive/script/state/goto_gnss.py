#!/usr/bin/env python3
import rospy
import smach
from geographic_msgs.msg import GeoPointStamped  
from gnc_executive.srv import GetNextCoordinate
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from robot_localization.srv import FromLL, FromLLRequest
import actionlib

class GoToGNSS(smach.State):
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        #self.pub = rospy.Publisher("/gps_goal", GeoPointStamped, queue_size=10)
    
    def execute(self,userdata):

        goal_gnss = self.get_next_coordinate_client()
        if goal_gnss is None:
            rospy.loginfo("Failed to get next coordinate.")
            return 'failed'

        rospy.loginfo(f"Navigating to {goal_gnss.position.latitude} {goal_gnss.position.longitude}...")
        
        rospy.wait_for_service('/fromLL')
        
        gps2utm = rospy.ServiceProxy('/fromLL', FromLL)

        request = FromLLRequest()
        request.ll_point.latitude = goal_gnss.position.latitude
        request.ll_point.longitude = goal_gnss.position.longitude
        
        response = gps2utm(request)

        goal = self.create_move_base_goal(response)
        
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        client.send_goal(goal)

        res = client.wait_for_result() #Blocking, client.get_state() for result
        
        if(client.get_state() == actionlib.GoalStatus.SUCCEEDED):
            return 'succeeded'
        else:
            return 'failed'
        
    
    def create_move_base_goal(self,response):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"

        goal.target_pose.pose.position.x = response.map_point.x
        goal.target_pose.pose.position.y = response.map_point.y
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.w = 1.0

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


