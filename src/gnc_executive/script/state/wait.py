import smach
import rospy

from fiducial_msgs.msg import FiducialTransformArray
import time

class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["complete"])

    def execute(self, userdata):
        time.sleep(3)
        return "complete"


