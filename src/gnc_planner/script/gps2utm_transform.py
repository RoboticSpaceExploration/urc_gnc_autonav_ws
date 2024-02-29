import rospy
import tf
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import utm

class GPSGoalNode:
    def __init__(self):
        rospy.init_node('gps_goal_node', anonymous=True)

        self.listener = tf.TransformListener()
        self.gps_sub = rospy.Subscriber("/gps_goal", GeoPointStamped, self.gps_callback)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server(rospy.Duration(5))

    def gps_callback(self, gps_msg):
        utm_point = self.gps_to_utm(gps_msg)

        # Ensure the transform from 'utm' to 'map' is available
        self.listener.waitForTransform('utm', 'map', rospy.Time(0), rospy.Duration(1.0))

        try:
            (trans, rot) = self.listener.lookupTransform('utm', 'map', rospy.Time(0))
            # Here we adjust the goal position by subtracting the transformation
            goal = self.create_move_base_goal(utm_point.pose.position.x - trans[0], utm_point.pose.position.y - trans[1])
            self.send_move_base_goal(goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)

    def gps_to_utm(self, gps_msg):
        utm_conversion = utm.from_latlon(gps_msg.position.latitude, gps_msg.position.longitude)
        utm_point = PoseStamped()
        utm_point.header.stamp = rospy.Time.now()
        utm_point.header.frame_id = 'utm'
        utm_point.pose.position.x = utm_conversion[0]
        utm_point.pose.position.y = utm_conversion[1]
        utm_point.pose.orientation.w = 1.0
        return utm_point

    def create_move_base_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # Changed to 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
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

