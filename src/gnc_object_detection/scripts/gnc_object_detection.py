import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imgshow("Zed Camera", cv_image)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print(e)

def main():
    rospy.init_node('zed_camera_viewer', anonymous=True)
    rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()