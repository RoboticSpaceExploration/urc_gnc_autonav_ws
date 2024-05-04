#!/usr/bin/env python
import rospy
import subprocess
from gnc_ground.srv import LaunchNode, LaunchNodeResponse

def handle_launch_node(req):
    """Service to launch a ROS node specified by a launch file."""
    try:
        # Attempt to launch the ROS node via subprocess
        # Define the path to the ROS setup file
        ros_setup_script = '/opt/ros/noetic/setup.bash'  # Adjust as per your ROS installation

        # Command to source the ROS setup file and then run roslaunch
        command = f"source {ros_setup_script} && source ~/Desktop/urc_gnc_autonav_ws/devel/setup.sh && nohup roslaunch {req.launch_file} &> /dev/null"
        
        # Start the subprocess to run the command within the bash shell
        process = subprocess.Popen(command, shell=True, executable='/bin/bash')
        
        # Wait for the command to complete

        return LaunchNodeResponse(True, "Node launched successfully")
    except Exception as e:
        rospy.logerr("Failed to launch: %s", str(e))
        return LaunchNodeResponse(False,"Failed to launch")

def launch_node_server():
    rospy.init_node('launch_node_service')
    s = rospy.Service('launch_node', LaunchNode, handle_launch_node)
    rospy.loginfo("Ready to launch nodes.")
    rospy.spin()

if __name__ == '__main__':
    launch_node_server()

