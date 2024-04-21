#!/usr/bin/env python3
import rospy
import tf
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import utm
from gnc_executive.srv import GPS2UTM,GPS2UTMResponse

class GPSGoalNode:
    def __init__(self):
        rospy.init_node('gps_goal_node', anonymous=True)

        self.listener = tf.TransformListener()
        #self.gps_sub = rospy.Subscriber("/gps_goal", GeoPointStamped, self.gps_callback)

        convert_utm = rospy.ServiceProxy('robot_localization/fromLL', FromLL)

        s = rospy.Service('gps2utm', GPS2UTM, self.execute)
        rospy.spin()

        rospy.loginfo("Initialized")

    def execute(self,req):
        rospy.loginfo("Executing")
        lat = req.lat
        lon = req.lon
        utm_point = self.gps_to_utm(lat,lon)

        self.listener.waitForTransform('utm', 'map', rospy.Time(0), rospy.Duration(1.0))

        try:
            (trans, rot) = self.listener.lookupTransform('utm', 'map', rospy.Time(0))
            utm_result_x = utm_point.pose.position.x - trans[0]
            utm_result_y = utm_point.pose.position.y - trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)

        return GPS2UTMResponse(utm_result_x, utm_result_y)

    def gps_callback(self, gps_msg):
        rospy.loginfo("Callback")
        

    def gps_to_utm(self, lat, lon):
        utm_conversion = utm.from_latlon(lat,lon)
        utm_point = PoseStamped()
        utm_point.header.stamp = rospy.Time.now()
        utm_point.header.frame_id = 'utm'
        utm_point.pose.position.x = utm_conversion[0]
        utm_point.pose.position.y = utm_conversion[1]
        utm_point.pose.orientation.w = 1.0
        return utm_point

if __name__ == '__main__':
    try:
        gps_goal_node = GPSGoalNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

