import rospy
from fiducial_msgs.msg import FiducialTransformArray

class ARUCOUtil():

    def __init__(self):
        self.tag_count = -1
    
    def count_tags(self):
        sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback)
        while not rospy.is_shutdown() and self.tag_count == -1:
            continue
        return self.tag_count
    
    def callback(self,msg):
        self.tag_count = len(msg.transforms)

#if __init__ == "main":
#rospy.init_node("test_node")
#au = ARUCOUtil()
#a = au.count_tags()
#print(a)
