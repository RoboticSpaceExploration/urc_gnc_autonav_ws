#!/usr/bin/env python3

import rospy
import tf
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf.transformations import quaternion_from_euler
import utm

class GPSGoalNode:
    def __init__(self):
        rospy.init_node('gps_goal_node', anonymous=True)

        # Create a TransformListener to get the transformations
        self.listener = tf.TransformListener()

        # Create a subscriber to GPS coordinates
        self.gps_sub = rospy.Subscriber("/gps_goal", GeoPointStamped, self.gps_callback)

        # Create a MoveBaseAction client to send goals to move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server(rospy.Duration(5))

    def gps_callback(self, gps_msg):
        # Transform the GPS coordinates to UTM
        utm_point = self.gps_to_utm(gps_msg)

        # Wait for the transform to be available
        self.listener.waitForTransform('utm', 'base_link', rospy.Time(0), rospy.Duration(1.0))

        # Transform UTM to base_link
        try:

            (trans,rot) = self.listener.lookupTransform('utm', 'map', rospy.Time(0))
            goal = self.create_move_base_goal(utm_point.pose.position.x - trans[0], utm_point.pose.position.y - trans[1])
            self.send_move_base_goal(goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)
    import utm

    def gps_to_utm(self, gps_msg):
        # Convert latitude and longitude to UTM coordinates
        utm_conversion = utm.from_latlon(gps_msg.position.latitude, gps_msg.position.longitude)
        utm_point = PoseStamped()
        utm_point.header.stamp = rospy.Time.now()
        utm_point.header.frame_id = 'utm'
        utm_point.pose.position.x = utm_conversion[0]  # UTM easting
        utm_point.pose.position.y = utm_conversion[1]  # UTM northing
        utm_point.pose.orientation.w = 1.0
        return utm_point

    def create_move_base_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0 # Facing forward
        return goal

    def send_move_base_goal(self, goal):
        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
        else:
            return self.move_base_client.get_result()

if __name__ == '__main__':
    try:
        gps_goal_node = GPSGoalNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

