#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

def publish_empty_costmap():
    rospy.init_node('empty_global_costmap_publisher')

    # Define the publisher
    pub = rospy.Publisher('/global_costmap', OccupancyGrid, queue_size=10)

    # Create the OccupancyGrid message
    grid = OccupancyGrid()
    grid.header = Header()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.1 # meters/cell
    grid.info.width = 100  # cells
    grid.info.height = 100  # cells
    grid.info.origin.position.x = -5.0  # Adjust origin to center the grid
    grid.info.origin.position.y = -5.0
    grid.info.origin.orientation.w = 1.0

    # Initialize all cells to 0 (free space)
    grid.data = [0] * int((100.0 * 100.0))  # 1km x 1km grid with 0.1m resolution

    # Publish the grid
    rate = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        pub.publish(grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_empty_costmap()
    except rospy.ROSInterruptException:
        pass

