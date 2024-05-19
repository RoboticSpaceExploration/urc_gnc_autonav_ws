#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import yaml

# Init variables
y_pos = 0.0
x_pos = 0.0
x_vel = 0.0
x_vel_time = 0.0
frequency = 0.0
delay = 0.0
yaw_offset = 0.0
magnetic_declination_radians = 0.0
zero_altitude = False
broadcast_utm_transform = False
publish_filtered_gps = False
use_odometry_yaw = False
wait_for_datum = False

def get_params():
    global x_vel, x_vel_time, frequency, delay, yaw_offset, magnetic_declination_radians
    global zero_altitude, broadcast_utm_transform, publish_filtered_gps, use_odometry_yaw, wait_for_datum

    x_vel = 1.0 #rospy.get_param('/outdoor_waypoint_nav/x_vel')
    x_vel_time = 8 #rospy.get_param('/outdoor_waypoint_nav/x_vel_time')
    frequency = rospy.get_param('/navsat_transform/frequency')
    delay = 0.0 #rospy.get_param('/navsat_transform/delay')
    yaw_offset = rospy.get_param('/navsat_transform/yaw_offset')
    magnetic_declination_radians = rospy.get_param('/navsat_transform/magnetic_declination_radians')
    zero_altitude = rospy.get_param('/navsat_transform/zero_altitude')
    broadcast_utm_transform = rospy.get_param('/navsat_transform/broadcast_utm_transform')
    publish_filtered_gps = True #rospy.get_param('/navsat_transform/publish_filtered_gps')
    use_odometry_yaw = False  #rospy.get_param('/navsat_transform/use_odometry_yaw')
    wait_for_datum = False #rospy.get_param('/navsat_transform/wait_for_datum')

def write_params(path_to_param_file, heading_err):
    params = {
        'navsat_transform': {
            'frequency': frequency,
            'delay': round(delay, 1),
            'magnetic_declination_radians': round(magnetic_declination_radians + heading_err, 5),
            'yaw_offset': round(yaw_offset, 5),
            'zero_altitude': zero_altitude,
            'broadcast_utm_transform': broadcast_utm_transform,
            'publish_filtered_gps': publish_filtered_gps,
            'use_odometry_yaw': use_odometry_yaw,
            'wait_for_datum': wait_for_datum
        }
    }

    with open(path_to_param_file, 'w') as params_file:
        yaml.dump(params, params_file, default_flow_style=False)

def filtered_odom_cb(odom_msgs):
    global y_pos, x_pos
    y_pos = odom_msgs.pose.pose.position.y
    x_pos = odom_msgs.pose.pose.position.x

if __name__ == '__main__':
    rospy.init_node('calibrate_heading')
    rospy.loginfo("Initiated calibration node")

    sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, filtered_odom_cb)
    pub_vel = rospy.Publisher('/gnc_robot/gnc_drive_velocity_controller/cmd_vel', Twist, queue_size=100)
    pub_calibration_node_ended = rospy.Publisher('/mag/calibrate_status', Bool, queue_size=100)

    get_params()
    rospy.logwarn("PLEASE ENSURE YOU HAVE MIN. %.1f m OF CLEAR SPACE IN FRONT OF YOUR ROBOT FOR CALIBRATION", (x_vel * x_vel_time))

    pub_rate = 20
    num_vel_msgs = int(x_vel_time * pub_rate)
    rate = rospy.Rate(pub_rate)

    velmsg = Twist()
    velmsg.linear.x = x_vel
    velmsg.angular.z = 0.0

    for _ in range(num_vel_msgs):
        pub_vel.publish(velmsg)
        rate.sleep()

    rospy.sleep(2)

    #rospy.spin_once()
    heading_error = math.atan2(y_pos, x_pos)
    rospy.loginfo("Detected heading error of: %.1f Degrees", 180 / math.pi * heading_error)

    rospack = rospkg.RosPack()
    path = rospack.get_path('gnc_localization') + '/launch/navsat_params.yaml'
    rospy.loginfo("Writing calibration results to file...")
    write_params(path, heading_error)
    rospy.loginfo("Wrote to param file: %s", path)

    rospy.loginfo("Returning to start...")
    velmsg.linear.x = -x_vel

    for _ in range(num_vel_msgs):
        pub_vel.publish(velmsg)
        rate.sleep()

    rospy.loginfo("Heading Calibration Complete")

    node_ended = Bool()
    node_ended.data = True
    pub_calibration_node_ended.publish(node_ended)

    rospy.loginfo("Ending Node...")
    rospy.logwarn("PLEASE RESTART YOUR EKF NODES TO APPLY NEW CALIBRATION PARAMETERS.")
    rospy.signal_shutdown("Calibration complete")

