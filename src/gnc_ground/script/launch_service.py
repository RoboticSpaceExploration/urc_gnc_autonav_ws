#!/usr/bin/env python
import rospy
import subprocess
from my_ros.srv import Launch, LaunchResponse

def handle_launch(req):
    try:
        subprocess.Popen(['roslaunch', req.launch_file])
        return LaunchResponse(True)
    except Exception as e:
        rospy.logerr("Failed to launch: %s" % str(e))
        return LaunchResponse(False)

def launch_server():
    rospy.init_node('launch_server')
    s = rospy.Service('launch_node', Launch, handle_launch)
    rospy.spin()

if __name__ == "__main__":
    launch_server()

