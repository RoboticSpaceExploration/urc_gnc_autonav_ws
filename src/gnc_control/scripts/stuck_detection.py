#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class StuckDetector:
    def __init__(self):
        self.cmd_vel = None
        self.odom_vel = None
        self.stuck = False
        self.stuck_duration = rospy.get_param('~stuck_duration', 3.0)  # seconds
        self.velocity_threshold = rospy.get_param('~velocity_threshold', 0.1)  # m/s
        self.last_cmd_time = rospy.Time.now()
        
        rospy.Subscriber('/gnc_robot/gnc_drive_velocity_controller/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/zed2/zed_node/odom', Odometry, self.odom_callback)
        self.stuck_pub = rospy.Publisher('/is_stuck', Bool, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_stuck)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.last_cmd_time = rospy.Time.now()

    def odom_callback(self, msg):
        self.odom_vel = msg.twist.twist

    def check_stuck(self, event):
        if self.cmd_vel is None or self.odom_vel is None:
            return

        # Calculate the difference between the commanded and actual velocities
        cmd_speed = abs(self.cmd_vel.linear.x)
        odom_speed = abs(self.odom_vel.linear.x)

        print(cmd_speed,odom_speed)

        if cmd_speed > self.velocity_threshold and odom_speed < (cmd_speed * 0.1):  # Example: 10% of commanded speed
            if rospy.Time.now() - self.last_cmd_time > rospy.Duration(self.stuck_duration):
                self.stuck = True
        else:
            self.stuck = False

        # Publish the stuck status
        self.stuck_pub.publish(Bool(self.stuck))

if __name__ == '__main__':
    rospy.init_node('stuck_detector')
    detector = StuckDetector()
    rospy.spin()

