import rospy
import actionlib
import tf2_ros
from smach import State
from geometry_msgs.msg import Twist, PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from fiducial_msgs.msg import FiducialTransformArray

class GateTraverse(State):
    def __init__(self):
        State.__init__(self, outcomes=['traversed', 'failed'],input_keys=['markers'])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.cmd_vel_pub = rospy.Publisher('/gnc_robot/gnc_drive_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.detected_markers = []

    def execute(self, userdata):
        rospy.loginfo('Going through ARUCO tags')

        marker = userdata.markers
        direction_norm = 10000

        while(direction_norm > 0.25):

            print(direction_norm)
            # Get transform from base_link to fiducial
            try:
                bl_transform = self.tf_buffer.lookup_transform('base_link', "fiducial_"+str(marker.transforms[0].fiducial_id), rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to get transform from base_link to {}".format(marker.header.frame_id))
                return 'failed'
            
            transform = marker.transforms[0]

            # Transform fiducial position to base_link frame
            fiducial_point = PointStamped()
            fiducial_point.header.frame_id = marker.header.frame_id
            fiducial_point.point.x = transform.transform.translation.x
            fiducial_point.point.y = transform.transform.translation.y
            fiducial_point.point.z = transform.transform.translation.z
            fiducial_point_transformed = do_transform_point(fiducial_point, bl_transform)
            # Calculate direction vector from robot base to fiducial
            direction_vector = fiducial_point_transformed.point
            direction_norm = ((direction_vector.x ** 2) + (direction_vector.y ** 2)) ** 0.5

            # Set linear velocity towards the fiducial
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.3 * direction_vector.x 
            cmd_vel_msg.linear.y = 0.3 * direction_vector.y 

            # Publish cmd_vel command
            self.cmd_vel_pub.publish(cmd_vel_msg)

            
        return 'traversed'

