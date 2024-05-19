#!/usr/bin/env python
import rospy
import os
from gnc_ground.srv import KillNode, KillNodeResponse

def handle_kill_node(req):
    node_name = req.node_name
    try:
        command = "rosnode kill " + node_name
        os.system(command)
        return KillNodeResponse(True, "Node killed successfully: " + node_name)
    except Exception as e:
        return KillNodeResponse(False, "Failed to kill node: " + str(e))

def kill_node_server():
    rospy.init_node('kill_node_service')
    s = rospy.Service('kill_node', KillNode, handle_kill_node)
    rospy.loginfo("Ready to kill nodes.")
    rospy.spin()

if __name__ == "__main__":
    kill_node_server()

