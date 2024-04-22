#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

def publish_empty_costmap():
    rospy.init_node('empty_global_costmap_publisher')

    # Define the publisher
    pub = rospy.Publisher('/global_costmap', OccupancyGrid, queue_size=10)

    # Create the OccupancyGrid message
    resolution = 0.5
    grid = OccupancyGrid()
    grid.header = Header()
    grid.header.frame_id = "map"
    grid.info.resolution = resolution # meters/cell
    grid.info.width = 4000  # cells
    grid.info.height = 4000  # cells
    grid.info.origin.position.x = -1000  # Adjust origin to center the grid
    grid.info.origin.position.y = -1000
    grid.info.origin.orientation.w = 1.0

    # Initialize all cells to 0 (free space)
    grid.data = [-1] *  int(4000*4000)
    # Publish the grid
    rate = rospy.Rate(0.1)  # 1Hz
    while not rospy.is_shutdown():
        pub.publish(grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_empty_costmap()
    except rospy.ROSInterruptException:
        pass

