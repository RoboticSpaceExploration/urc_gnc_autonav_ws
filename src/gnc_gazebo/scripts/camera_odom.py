#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry

# Global variable to store the last processed timestamp
last_processed_time = None
# Define a minimum time difference threshold (in seconds)
#time_threshold = 0.05
time_threshold = 0.02

def odom_callback(msg):
    global last_processed_time

    # Extract timestamp from the message
    current_time = msg.header.stamp.to_sec()

    # Check if the last processed time is set and the difference is below the threshold
    if last_processed_time is not None and (current_time - last_processed_time) < time_threshold:
        # Ignore this message
        return

    # Update the last processed timestamp
    last_processed_time = current_time

    # Extract position and orientation from the odometry message
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    # Publish the transform
    br.sendTransform(
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
        #rospy.Time.now(),
        msg.header.stamp,
        "base_link",
        "odom"
    )

def odom_listener():
    # Initialize the node
    rospy.init_node('odom_to_tf_publisher')

    # Subscribe to the odometry topic
    rospy.Subscriber('/zed2/zed_node/odom', Odometry, odom_callback)

    # Spin to keep the script from exiting
    rospy.spin()

if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass

