#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3

# Global variables to store the total traveled distance
total_distance_traveled = 0.0
last_position = None
last_time = None

def callback(data):
    global last_position
    global total_distance_traveled
    global last_time
    
    current_position = Vector3(data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    
    if last_position is None:
        # Initialize last_position if not already set
        last_position = current_position
        last_time = data.header.stamp
        return

    

    curr_time = data.header.stamp

    dt = (curr_time - last_time).to_sec()

    pos_delta = Vector3( current_position.x - last_position.x,
                         current_position.y - last_position.y,
                         current_position.z - last_position.z)
    twist = TwistWithCovarianceStamped()
    twist.twist.twist.linear = Vector3((pos_delta.x)/dt, pos_delta.y/dt,pos_delta.z/dt)

    twist.twist.covariance[0] = 1.5
    twist.twist.covariance[7] = 1.5
    twist.twist.covariance[14] = 1.5

    last_position = current_position
    last_time = curr_time
    twist.header = data.header
    # Create a new Odometry message
    #modified_odom = Odometry()
    #modified_odom.header = data.header
    #modified_odom.child_frame_id = data.child_frame_id
    #modified_odom.twist = twist
    
    # Add additional covariance to pose covariance
    #modified_odom.pose.covariance = [x + additional_covariance for x in data.pose.covariance]

    #modified_odom.pose.covariance = [ 0 for x in data.pose.covariance]

    #modified_odom.pose.covariance[0] +=  additional_covariance 
    #modified_odom.pose.covariance[7] +=  additional_covariance
    #modified_odom.pose.covariance[14] +=  additional_covariance
    
    # Publish the modified odometry message
    pub.publish(twist)

def listener():
    rospy.init_node('odom_twist', anonymous=True)
    
    # Subscribe to the odometry topic
    rospy.Subscriber("/zed2/zed_node/odom", Odometry, callback)
    
    # Publisher for the modified odometry messages
    global pub
    pub = rospy.Publisher('/zed2/zed_node/odom/twist', TwistWithCovarianceStamped, queue_size=10)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()

