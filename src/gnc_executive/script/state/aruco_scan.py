import smach
import rospy

from fiducial_msgs.msg import FiducialTransformArray

class ARUCOScan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'], output_keys=['markers'])
    def execute(self, userdata):
        rospy.loginfo('Performing scan')
        au = ARUCOUtil() 
        if(au.count_tags() >= 1):
            userdata.markers = au.markers
            return 'found'
        return 'not_found'


class ARUCOUtil():

    def __init__(self):
        self.tag_count = -1
        self.markers = []
    def count_tags(self):
        sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback)
        while not rospy.is_shutdown() and self.tag_count == -1:
            continue
        
        return self.tag_count
    
    def callback(self,msg):
        self.tag_count = len(msg.transforms)
        self.markers = msg

