import rospy
import actionlib
from smach import State
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from aruco_msgs.msg import MarkerArray  # Assuming aruco_msgs is the message type

class GateTraverse(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed', 'failed'])
        #self.aruco_sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.aruco_callback)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.detected_markers = []

    def aruco_callback(self, msg):
        self.detected_markers = msg.markers

    def calculate_goal(self, marker):
        goal_distance = 3.0  # meters

        # Convert quaternion to Euler angles to get orientation
        orientation = [aruco_pose.pose.orientation.x,
                       aruco_pose.pose.orientation.y,
                       aruco_pose.pose.orientation.z,
                       aruco_pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation)

        # Calculate goal point that is 3 meters away from the ARUCO tag
        goal_x = aruco_pose.pose.position.x + goal_distance * rospy.cos(euler[2])
        goal_y = aruco_pose.pose.position.y + goal_distance * rospy.sin(euler[2])

        # Create a new PoseStamped for the goal, assuming the same frame_id and orientation as the ARUCO tag
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = aruco_pose.header.frame_id
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, euler[2]))

        return goal_pose

    def execute(self, userdata):
        rospy.loginfo('Going through ARUCO tags')
        self.move_base_client.wait_for_server()

        if not self.detected_markers:
            rospy.loginfo('No ARUCO tags detected')
            return 'failed'

        for marker in self.detected_markers:
            goal = self.calculate_goal(marker)
            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()

            if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo('Successfully reached goal near ARUCO tag')
            else:
                rospy.loginfo('Failed to reach goal near ARUCO tag')
                return 'failed'

        return 'completed'

