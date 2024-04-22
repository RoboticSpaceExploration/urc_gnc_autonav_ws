#!/usr/bin/env python
import rospy
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

class OdometryFusion:
    def __init__(self):
        rospy.init_node('odometry_fusion_node', anonymous=True)

        # Subscribers
        self.gps_sub = rospy.Subscriber('/odometry/gps', Odometry, self.gps_callback)
        self.imu_sub = rospy.Subscriber('/zed2/zed_node/imu/data', Imu, self.imu_callback)

        
        # TF Broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher('/odometry/filtered', Odometry, queue_size=10)
        # Data storage
        self.current_gps = None
        self.current_orientation = None

    def gps_callback(self, msg):
        self.current_gps = msg

    def imu_callback(self, msg):
        self.current_orientation = msg.orientation

    def publish_tf(self):
        if self.current_gps is not None and self.current_orientation is not None:
            t = TransformStamped()
            
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            
            # Assuming the GPS provides latitude, longitude, and altitude
            # Convert GPS data to XYZ coordinates here if necessary
            # For simplicity, this example will just reuse the GPS position data
            t.transform.translation.x = self.current_gps.pose.pose.position.x
            t.transform.translation.y = self.current_gps.pose.pose.position.y
            t.transform.translation.z = self.current_gps.pose.pose.position.z
            
            # Use the orientation from the IMU
            t.transform.rotation = self.current_orientation
            
            # Send the transform
            self.br.sendTransform(t)

            odom = Odometry()
            odom.header = t.header
            odom.child_frame_id = "odom"
            odom.pose.pose.position = self.current_gps.pose.pose.position
            odom.pose.pose.orientation = self.current_orientation

            odom.twist.twist.linear.x = 0
            odom.twist.twist.linear.y = 0
            odom.twist.twist.linear.z = 0
            odom.twist.twist.angular.x = 0
            odom.twist.twist.angular.y = 0
            odom.twist.twist.angular.z = 0

            self.odom_pub.publish(odom)
    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.publish_tf()
            rate.sleep()

if __name__ == '__main__':
    odometry_fusion = OdometryFusion()
    odometry_fusion.run()
