#!/usr/bin/env python
import rospy
import subprocess
import actionlib
from gnc_ground.msg import LaunchNodeAction, LaunchNodeFeedback, LaunchNodeResult

class LaunchNodeActionServer:
    _feedback = LaunchNodeFeedback()
    _result = LaunchNodeResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer('launch_node', LaunchNodeAction, self.execute_cb, False)
        self._as.start()

    def execute_cb(self, goal):
        """Action callback to launch a ROS node specified by a launch file."""
        try:
            ros_setup_script = '/opt/ros/noetic/setup.bash'  # Adjust as per your ROS installation
            command = f"source {ros_setup_script} && source ~/Desktop/urc_gnc_autonav_ws/devel/setup.sh && nohup roslaunch {goal.launch_file} &> /dev/null &"

            # Start the subprocess to run the command within the bash shell
            process = subprocess.Popen(command, shell=True, executable='/bin/bash')

            # Immediately return success without waiting for the process to complete
            self._result.success = True
            self._result.message = "Node launch command executed"
            self._as.set_succeeded(self._result)
        except Exception as e:
            rospy.logerr("Failed to launch: %s", str(e))
            self._result.success = False
            self._result.message = f"Failed to launch: {str(e)}"
            self._as.set_aborted(self._result)

if __name__ == '__main__':
    rospy.init_node('launch_node_action_server')
    server = LaunchNodeActionServer()
    rospy.loginfo("Launch Node Action Server ready.")
    rospy.spin()

