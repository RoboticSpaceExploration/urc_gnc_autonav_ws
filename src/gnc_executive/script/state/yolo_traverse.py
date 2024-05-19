#!/usr/bin/env python

import rospy
import actionlib
import tf2_ros
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from ultralytics_ros.msg import YoloResult  # Replace with the actual message type and package
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class YOLOFollower(State):
    def __init__(self):
        State.__init__(self, outcomes=['traversed', 'failed'], input_keys=['markers'])
        self.kp = rospy.get_param('~kp', 0.1)  # Proportional gain for the control input
        self.stop_distance_threshold = rospy.get_param('~stop_distance_threshold', 100)  # Threshold for stopping
        self.image_width = None  # Initialize image width as None

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        self.detection_sub = rospy.Subscriber('/yolo_result', YoloResult, self.detection_callback)
        self.image_sub = rospy.Subscriber('/zed2/zed_node/left_raw/image_raw_color', Image, self.image_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/gnc_robot/gnc_drive_velocity_controller/cmd_vel', Twist, queue_size=10)

        self.marker_detected = False
        self.bbox_center_x = 0
        self.bbox_size_y = 0

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Get the width of the image
            self.image_width = cv_image.shape[1]
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def detection_callback(self, msg):
        if not msg.detections or self.image_width is None or len(msg.detections.detections) == 0:
            self.marker_detected = False
            return  # No detections or image width not set

        # Take the first detection (you can enhance this by selecting the best detection if needed)
        detection = msg.detections.detections[0]
        self.bbox_center_x = detection.bbox.center.x
        self.bbox_size_y = detection.bbox.size_y
        self.marker_detected = True

    def execute(self, userdata):
        rospy.loginfo('Approaching detected object')

        while not rospy.is_shutdown():
            if not self.marker_detected:
                rospy.logwarn("No marker detected")
                return 'failed'

            # Calculate the error (distance from the center of the image)
            error_x = self.bbox_center_x - (self.image_width / 2)

            # Initialize control input
            twist = Twist()

            # Check if the object is close enough to stop
            if self.bbox_size_y > self.stop_distance_threshold:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return 'traversed'
            else:
                # Calculate control input
                twist.linear.x = 0.2  # Forward movement, adjust as needed
                twist.angular.z = -self.kp * error_x
                self.cmd_vel_pub.publish(twist)

        return 'failed'

def main():
    rospy.init_node('yolo_follower_smach')

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['traversed'])

    # Add states to the state machine
    with sm:
        StateMachine.add('YOLO_TRAVERSE', YOLOFollower(), transitions={'traversed':'traversed', 'failed':'traversed'}, remapping={'markers':'markers'})

    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()

