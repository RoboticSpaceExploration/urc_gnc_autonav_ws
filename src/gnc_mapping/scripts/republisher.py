#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def odometry_callback(msg):
    """
    Callback function for the /odometry/filtered topic subscriber.
    Extracts the pose and covariance data and publishes it to the /odometry/filtered/pose topic.
    """
    # Create a PoseWithCovarianceStamped message
    pose_msg = PoseWithCovarianceStamped()

    # Fill the header with the incoming odometry message's header
    pose_msg.header = msg.header

    # Copy the pose data
    pose_msg.pose.pose = msg.pose.pose

    # Copy the covariance matrix
    pose_msg.pose.covariance = msg.pose.covariance

    # Publish the pose message
    pose_pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        # Initialize the ROS Node
        rospy.init_node('odometry_to_pose', anonymous=True)

        # Create a publisher for the PoseWithCovarianceStamped message
        pose_pub = rospy.Publisher('/odometry/filtered/pose', PoseWithCovarianceStamped, queue_size=10)

        # Create a subscriber to the /odometry/filtered topic
        rospy.Subscriber('/zed2/odom/sample', Odometry, odometry_callback)

        # Keep the node running until it is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

