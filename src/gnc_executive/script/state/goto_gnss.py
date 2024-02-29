#!/usr/bin/env python3
import rospy
import smach
from geographic_msgs.msg import GeoPointStamped  
from gnc_executive.srv import GetNextCoordinate
from move_base_msgs.msg import MoveBaseActionResult


class GoToGNSS(smach.State):
    def __init__(self, waypoint):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.gps_coordinate = waypoint
        # Initialize Publisher and Subscriber with correct message types and topics
        self.pub = rospy.Publisher("/gps_goal", GeoPointStamped, queue_size=10)
        # Assuming move_base publishes a simple String on success, adjust as necessary
        self.success_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.move_base_callback)
        self.move_base_success = False
    def execute(self, userdata):
        # Reset success flag
        self.move_base_success = False

        goal_gnss = self.get_next_coordinate_client()
        if goal_gnss is None:
            rospy.loginfo("Failed to get next coordinate.")
            return 'failed'

        rospy.loginfo(f"Navigating to {goal_gnss.position.latitude} {goal_gnss.position.longitude}...")
        goal_gnss.header.stamp = rospy.Time.now()
        self.pub.publish(goal_gnss)

        # Wait for move_base success
        while not rospy.is_shutdown() and not self.move_base_success:
            rospy.sleep(0.1)  # Sleep to yield control to ROS

        return 'succeeded' if self.move_base_success else 'failed'

    def move_base_callback(self, msg):
        if msg.status.status == 3:
            self.move_base_success = True

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


