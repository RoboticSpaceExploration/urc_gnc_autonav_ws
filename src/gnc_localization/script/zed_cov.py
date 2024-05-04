#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry

# Global variables to store the total traveled distance
total_distance_traveled = 0.0
last_position = None

def callback(data):
    global last_position
    global total_distance_traveled
    
    # Get current position
    current_position = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    
    if last_position is None:
        # Initialize last_position if not already set
        last_position = current_position
        return

    # Calculate the distance moved in this frame
    distance_moved = math.sqrt((current_position[0] - last_position[0])**2 +
                               (current_position[1] - last_position[1])**2 +
                               (current_position[2] - last_position[2])**2)
    
    # Update the total distance traveled
    total_distance_traveled += distance_moved
    
    # Update last_position
    last_position = current_position
    
    # Additional covariance to add based on total distance (10% of the total distance traveled)
    #additional_covariance = total_distance_traveled * 0.1 
    additional_covariance = 25 
    # Create a new Odometry message
    modified_odom = Odometry()
    modified_odom.header = data.header
    modified_odom.child_frame_id = data.child_frame_id
    modified_odom.pose = data.pose
    modified_odom.twist = data.twist
    
    # Add additional covariance to pose covariance
    #modified_odom.pose.covariance = [x + additional_covariance for x in data.pose.covariance]

    modified_odom.pose.covariance = [ 0 for x in data.pose.covariance]

    modified_odom.pose.covariance[0] +=  additional_covariance 
    modified_odom.pose.covariance[7] +=  additional_covariance
    modified_odom.pose.covariance[14] +=  additional_covariance
    
    # Publish the modified odometry message
    pub.publish(modified_odom)

def listener():
    rospy.init_node('odom_total_covariance_adjuster', anonymous=True)
    
    # Subscribe to the odometry topic
    rospy.Subscriber("/zed2/zed_node/odom", Odometry, callback)
    
    # Publisher for the modified odometry messages
    global pub
    pub = rospy.Publisher('/zed2/zed_node/odom/scaled', Odometry, queue_size=10)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()

