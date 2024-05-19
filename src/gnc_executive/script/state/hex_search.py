import smach
import rospy
import math
import tf2_ros
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from fiducial_msgs.msg import FiducialTransformArray
import actionlib

class HexSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'], output_keys=["markers"])
        self.step_size = 5  # Size of each step in the hexagonal search (meters)
        self.max_hex_steps = 10  # Maximum number of hexagonal steps
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.aruco_util = ARUCOUtil()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self, userdata):
        rate = rospy.Rate(10)
        x = y = None
        transform = None
        rospy.loginfo('Performing hexagonal search')

        while transform is None:
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF error: {e}")
            rate.sleep()

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        directions = [(1, 0), (0.5, math.sqrt(3)/2), (-0.5, math.sqrt(3)/2),
                      (-1, 0), (-0.5, -math.sqrt(3)/2), (0.5, -math.sqrt(3)/2)]

        for step in range(1, self.max_hex_steps + 1):
            for direction in directions:
                goal_x = x + step * self.step_size * direction[0]
                goal_y = y + step * self.step_size * direction[1]

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = goal_x
                goal.target_pose.pose.position.y = goal_y
                
                quaternion = quaternion_from_euler(0, 0, math.atan2(direction[1], direction[0]))
                goal.target_pose.pose.orientation.x = quaternion[0]
                goal.target_pose.pose.orientation.y = quaternion[1]
                goal.target_pose.pose.orientation.z = quaternion[2]
                goal.target_pose.pose.orientation.w = quaternion[3]

                rospy.loginfo(f"Sending goal to position: ({goal_x}, {goal_y})")
                client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                client.wait_for_server()
        
                client.send_goal(goal)
        
                res = client.wait_for_result()  # Blocking, client.get_state() for result
        
                found, userdata.markers = self.check_for_tags()
                if found:
                    rospy.loginfo("Found ARUCO tags.")
                    return 'found'
                
                x = goal_x
                y = goal_y
        
        rospy.loginfo("No ARUCO tags found.")
        return 'not_found'
    
    def check_for_tags(self):
        markers = self.aruco_util.count_tags()
        if markers > 0:
            return True, markers
        else:
            return False, None


class ARUCOUtil():
    def __init__(self):
        self.tag_count = -1
        self.markers = []
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback)

    def count_tags(self):
        while not rospy.is_shutdown() and self.tag_count == -1:
            continue
        return self.tag_count
    
    def callback(self, msg):
        self.tag_count = len(msg.transforms)
        self.markers = msg


if __name__ == '__main__':
    rospy.init_node('hex_search_state')

    sm = smach.StateMachine(outcomes=['found', 'not_found'])
    with sm:
        smach.StateMachine.add('HEX_SEARCH', HexSearch(), 
                               transitions={'found': 'found', 
                                            'not_found': 'not_found'})

    outcome = sm.execute()

