import rospy
import tf2_ros
from smach import State
from geometry_msgs.msg import Twist, PointStamped
from tf2_ros import Buffer, TransformListener
from fiducial_msgs.msg import FiducialTransformArray
import math

class GateTraverse(State):
    def __init__(self):
        State.__init__(self, outcomes=['traversed', 'failed'], input_keys=['markers'])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.cmd_vel_pub = rospy.Publisher('/gnc_robot/gnc_drive_velocity_controller/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Approaching ARUCO Tag')

        markers = userdata.markers
        if not markers.transforms:
            rospy.logerr("No markers received")
            return 'failed'

        marker_id = markers.transforms[0].fiducial_id
        distance = float('inf')
        odom_transform = None
        ttl = 20
        while not rospy.is_shutdown() and distance > 1.50:
            odtf_prime = None
            try:
                # Get the transform from odom to fiducial marker
                odtf_prime = self.tf_buffer.lookup_transform('base_link', f'fiducial_{marker_id}', rospy.Time(0))
                ttl = 20
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                #rospy.logerr(f"Failed to get transform from odom to fiducial_{marker_id}")
                #return 'failed'
                ttl = ttl - 1 
                if(ttl < 0):
                    return 'failed'

            if(odtf_prime != None):
                odom_transform = odtf_prime
            # Extract fiducial position in the odom frame
            fiducial_point = PointStamped()
            fiducial_point.header.frame_id = odom_transform.header.frame_id
            fiducial_point.point = odom_transform.transform.translation

            # Calculate direction vector in the odom frame
            direction_vector = fiducial_point.point
            distance = (direction_vector.x ** 2 + direction_vector.y ** 2) ** 0.5
            angle_to_target = math.atan2(direction_vector.y, direction_vector.x)

            # Calculate control commands
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.05 * distance
            cmd_vel_msg.angular.z = 0.05 * angle_to_target

            self.cmd_vel_pub.publish(cmd_vel_msg)

        return 'traversed'

if __name__ == '__main__':
    rospy.init_node('gate_traverse_node')
    sm = StateMachine(outcomes=['traversed', 'failed'])
    with sm:
        StateMachine.add('GateTraverse', GateTraverse(), transitions={'traversed':'traversed', 'failed':'failed'})
    
    outcome = sm.execute()

