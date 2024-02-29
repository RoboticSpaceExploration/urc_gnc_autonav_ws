import math
import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

# Initialize ROS node
rospy.init_node('spiral_goal_sender_map_frame')

print("[*N] Start spiral...") 
# Define the action client to interact with the move_base server
client = SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

# Spiral parameters for the global map
angle = 0.0  # Initial angle in radians
radius_increment = 0.35  # How much to increase the radius after each loop
angle_increment = math.radians(30)  # Radians to increase the angle each time
current_radius = radius_increment  # Start with the first step away from the initial point

number_of_points = 100  # Define how many points you want in your spiral

initial_position = (0.0, 0.0)  # Define the initial position in the map frame

for _ in range(number_of_points):
    # Convert polar coordinates (radius and angle) to Cartesian coordinates (x, y)
    x = initial_position[0] + current_radius * math.cos(angle)
    y = initial_position[1] + current_radius * math.sin(angle)

    # Assuming the robot should face towards the next point in the spiral,
    # calculate orientation to face that direction
    orientation_q = quaternion_from_euler(0, 0, angle)

    # Setup the goal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = orientation_q[0]
    goal.target_pose.pose.orientation.y = orientation_q[1]
    goal.target_pose.pose.orientation.z = orientation_q[2]
    goal.target_pose.pose.orientation.w = orientation_q[3]

    # Send the goal to the move_base action server
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(5))  # Wait for a maximum of 5 seconds for the robot to reach the goal

    # Update the parameters for the next goal in the spiral
    angle += angle_increment
    current_radius += radius_increment

