#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry

# Global variables to store the total traveled distance

def callback(data):
    
    # Create a new Odometry message
    modified_odom = Odometry()
    modified_odom.header = data.header
    modified_odom.child_frame_id = data.child_frame_id
    modified_odom.pose = data.pose
    
    # Add additional covariance to pose covariance
    #modified_odom.pose.covariance = [x + additional_covariance for x in data.pose.covariance]

    modified_odom.pose.covariance = [ 0 for x in data.pose.covariance]

    modified_odom.pose.covariance[0] = 0.1 
    modified_odom.pose.covariance[7] = 0.1
    modified_odom.pose.covariance[14] = 0.1
    
    # Publish the modified odometry message
    pub.publish(modified_odom)

def listener():
    rospy.init_node('odom_total_covariance_adjuster', anonymous=True)
    
    # Subscribe to the odometry topic
    rospy.Subscriber("/odometry/gps", Odometry, callback)
    
    # Publisher for the modified odometry messages
    global pub
    pub = rospy.Publisher('/odometry/gps/scaled', Odometry, queue_size=10)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()

