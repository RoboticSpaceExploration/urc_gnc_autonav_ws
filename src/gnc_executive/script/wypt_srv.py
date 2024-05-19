#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import String
from gnc_executive.srv import LaunchDetectService, LaunchDetectServiceResponse

class LaunchDetectNode:
    def __init__(self):
        rospy.init_node('launch_detect_node')
        
        self.current_process = None
        self.current_mode = None

        # Service to handle launch requests
        self.service = rospy.Service('launch_detect_service', LaunchDetectService, self.handle_launch_request)

        # Subscriber to the scan mode topic
        rospy.Subscriber('/scan_mode', String, self.scan_mode_callback)

        rospy.loginfo("LaunchDetectNode initialized.")

    def scan_mode_callback(self, msg):
        rospy.loginfo(f"Received scan mode: {msg.data}")
        self.launch_detect_mode(msg.data)

    def handle_launch_request(self, req):
        mode = req.mode
        self.launch_detect_mode(mode)
        return LaunchDetectNodeResponse(success=True, message=f"Launched {mode} mode successfully")

    def launch_detect_mode(self, mode):
        if mode == self.current_mode:
            rospy.loginfo(f"{mode} mode is already running.")
            return

        if self.current_process:
            rospy.loginfo(f"Stopping the current {self.current_mode} process.")
            self.current_process.shutdown()
            self.current_process = None

        if mode == "aruco":
            launch_file = '/home/roselab/Desktop/urc_gnc_autonav_ws/src/gnc_waypoints/launch/gnc_waypoints.launch'
        elif mode == "yolo":
            launch_file = '/home/roselab/Desktop/urc_gnc_autonav_ws/src/ultralytics_ros/launch/tracker.launch'
        else:
            rospy.logwarn(f"Unknown mode: {mode}")
            return

        rospy.loginfo(f"Launching {mode} detection.")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.current_process = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self.current_process.start()
        self.current_mode = mode

if __name__ == '__main__':
    try:
        LaunchDetectNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

