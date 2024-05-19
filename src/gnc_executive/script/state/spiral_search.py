#!/usr/bin/env python

import rospy
import actionlib
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from fiducial_msgs.msg import FiducialTransformArray
import tf
import tf2_ros
import math
from dynamic_reconfigure.client import Client as DynamicReconfigureClient

class SpiralSearch(smach.State):
    def __init__(self, radius=10, steps=5):
        smach.State.__init__(self, outcomes=['found', 'not_found'], output_keys=["markers"])
        self.radius = radius
        self.steps = steps
        self.aruco_detected = False
        self.detected_fiducials = set()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.dyn_client = DynamicReconfigureClient('move_base/TrajectoryPlannerROS')
        self.userdata = None

        # Subscribe to ARUCO tag detections
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.aruco_callback)

        # Store original parameters
        #self.original_params = self.dyn_client.get_configuration()

    def aruco_callback(self, data):
        for transform in data.transforms:
            fiducial_id = transform.fiducial_id
            if fiducial_id not in self.detected_fiducials:
                self.aruco_detected = True
                self.detected_fiducials.add(fiducial_id)
                rospy.loginfo(f"ARUCO tag {fiducial_id} detected! Stopping the spiral movement.")
                self.markers = data
                break

    def get_initial_pose(self):
        rate = rospy.Rate(10.0)
        trans = None
        while trans is None:
            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
                return trans.transform.translation.x, trans.transform.translation.y
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Waiting for transform from map to base_link")
                rate.sleep()

    def square_spiral_corners(self):
        x, y = 0, 0
        dx, dy = 0, -1
        coordinates = [(x, y)]

        step_size = self.radius / self.steps  # Each step increases by radius / steps
        length = 1

        for _ in range(self.steps):
            for _ in range(2):
                for _ in range(length):
                    x += dx * step_size
                    y += dy * step_size
                    coordinates.append((x, y))
                dx, dy = -dy, dx  # Change direction
            length += 1

        return coordinates

    def move_to_goal(self, x, y):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.atan2(y, x))
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo(f"Sending goal: {goal}")
        self.client.send_goal(goal)

        # Continuously check if ARUCO tag is detected
        while not rospy.is_shutdown():
            if self.aruco_detected:
                self.client.cancel_goal()
                rospy.loginfo("Goal preempted due to ARUCO tag detection.")
                return False
            if self.client.wait_for_result(rospy.Duration(0.1)):
                return self.client.get_result()

    def update_move_base_params(self, turn_in_place):
        current_params = self.dyn_client.get_configuration()
        if turn_in_place:
            params = current_params.copy()
            params.update({
                'max_vel_x': 0.0,
                'min_vel_x': 0.0,
                'max_vel_y': 0.0,
                'min_vel_y': 0.0,
                'yaw_goal_tolerance': 0.5  # Update yaw goal tolerance to 0.5 rad
            })
        else:
            params = self.original_params  # Revert to original parameters
        self.dyn_client.update_configuration(params)

    def execute(self, userdata):
        # Update parameters to turn in place
        #self.update_move_base_params(turn_in_place=True)

        initial_x, initial_y = self.get_initial_pose()
        rospy.loginfo(f"Initial rover pose: x={initial_x}, y={initial_y}")

        coords = self.square_spiral_corners()

        for coord in coords:
            if self.aruco_detected:
                rospy.loginfo("ARUCO tag detected. Stopping the spiral trajectory.")
                userdata.markers = self.markers
                # Revert parameters after execution
                #self.update_move_base_params(turn_in_place=False)
                return 'found'
            target_x = initial_x + coord[0]
            target_y = initial_y + coord[1]
            result = self.move_to_goal(target_x, target_y)
            if result:
                rospy.loginfo(f"Reached goal: ({target_x}, {target_y})")
            else:
                rospy.logerr(f"Failed to reach goal: ({target_x}, {target_y})")

        # Revert parameters after execution
        #self.update_move_base_params(turn_in_place=False)
        return 'not_found'

def main():
    rospy.init_node('spiral_trajectory_node')

    radius = rospy.get_param('~radius', 10)  # Total radius in meters
    steps = rospy.get_param('~steps', 4)    # Number of update steps

    # Create and execute the state
    state = SpiralSearch(radius, steps)
    outcome = state.execute(None)
    rospy.loginfo(f"Spiral trajectory completed with outcome: {outcome}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

