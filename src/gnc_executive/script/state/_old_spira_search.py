import smach
import rospy
import math
import tf2_ros
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from fiducial_msgs.msg import FiducialTransformArray
import actionlib

class SpiralSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'], output_keys=["markers"])
        # Parameters for the square spiral
        self.step_size = 1.0  # Size of each step in the spiral (meters)
        self.max_spiral_steps = 5  # Maximum number of spiral steps
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.aruco_util = ARUCOUtil()
        # Initialize TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self, userdata):

        rate = rospy.Rate(10)
        x = y = None
        transform = None
        rospy.loginfo('Performing square spiral search')
        # Get the robot's current position using TF
        while transform == None:
            try:
            # Wait for the transform between the 'map' and 'base_link' frames
            #self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration(5.0))
                transform = self.tf_buffer.lookup_transform('map','base_link',  rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF error: {e}")
            rate.sleep()

        print(transform)
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        # Initialize other variables
        angle = 0.0  # Starting orientation
        side_length = self.step_size  # Initial side length
        direction = 1  # Initial direction (1 for counter-clockwise, -1 for clockwise)
        
        # Execute the spiral pattern
        for step in range(1, self.max_spiral_steps + 1):
            for i in range(4):  # Each step consists of 4 sides (square pattern)
                # Calculate the goal position
                goal_x = x + direction * side_length * math.cos(angle)
                goal_y = y + direction * side_length * math.sin(angle)
                
                # Create a MoveBaseGoal object
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = goal_x
                goal.target_pose.pose.position.y = goal_y
                
                # Set the orientation of the goal
                quaternion = quaternion_from_euler(0, 0, angle)
                goal.target_pose.pose.orientation.x = quaternion[0]
                goal.target_pose.pose.orientation.y = quaternion[1]
                goal.target_pose.pose.orientation.z = quaternion[2]
                goal.target_pose.pose.orientation.w = quaternion[3]
               

                rospy.loginfo(f"Sending goal to position: ({goal_x}, {goal_y})")
                client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                client.wait_for_server()
        
                client.send_goal(goal)
        
                res = client.wait_for_result() #Blocking, client.get_state() for result
        
                
                # Check if any ARUCO tags are found
                found, userdata.markers = self.check_for_tags()
                if found:
                    rospy.loginfo("Found ARUCO tags.")
                    return 'found'
                
                # Update the current position
                x = goal_x
                y = goal_y
                
                # Update the angle for the next side of the square
                angle += math.pi / 2  # Turn 90 degrees (pi/2 radians)
            
            # Increase the side length for the next spiral step
            side_length += self.step_size
            
            # Change direction for the next step (alternates between counter-clockwise and clockwise)
            direction *= -1
        
        # If no ARUCO tags are found after completing the spiral search
        rospy.loginfo("No ARUCO tags found.")
        return 'not_found'
    
    def check_for_tags(self):
        # Implement ARUCO tag detection here, using your ARUCO utility
        markers = self.aruco_util.count_tags()
        if markers > 0:
            return True, markers
        else:
            return False, None


class ARUCOUtil():

    def __init__(self):
        self.tag_count = -1
        self.markers = []

        sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback)
    def count_tags(self):
        while not rospy.is_shutdown() and self.tag_count == -1:
            continue
        
        return self.tag_count
    
    def callback(self,msg):
        self.tag_count = len(msg.transforms)
        self.markers = msg

if __name__ == '__main__':
    rospy.init_node('spiral_search_state')

    sm = smach.StateMachine(outcomes=['found', 'not_found'])
    with sm:
        smach.StateMachine.add('SPIRAL_SEARCH', SpiralSearch(), 
                               transitions={'found':'found', 
                                            'not_found':'not_found'})

    outcome = sm.execute()

