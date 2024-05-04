#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

class GPSRepublisher:
    def __init__(self):
        # Set up the subscriber and publisher
        self.subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        self.publisher = rospy.Publisher('/gps/fix/time', NavSatFix)

    def gps_callback(self, msg):
        # Copy the original message to republish
        new_msg = NavSatFix()
        new_msg.header.stamp = rospy.Time.now()  # Set the current ROS time in the header
        new_msg.header.frame_id = msg.header.frame_id
        new_msg.status = msg.status
        new_msg.latitude = msg.latitude
        new_msg.longitude = msg.longitude
        new_msg.altitude = msg.altitude
        new_msg.position_covariance = msg.position_covariance
        new_msg.position_covariance_type = msg.position_covariance_type

        # Publish the modified message
        self.publisher.publish(new_msg)

if __name__ == '__main__':
    rospy.init_node('gps_republisher')
    republisher = GPSRepublisher()
    rospy.spin()

