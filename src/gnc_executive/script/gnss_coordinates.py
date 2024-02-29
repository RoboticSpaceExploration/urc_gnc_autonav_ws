#!/usr/bin/env python3

import rospy
from geographic_msgs.msg import GeoPointStamped,GeoPoint
from std_msgs.msg import Header
from gnc_executive.srv import GetNextCoordinate, GetNextCoordinateResponse  # Update with your package name

# Path to the file containing GPS coordinates
file_path = '/home/napporo/Desktop/urc_gnc_autonav_ws/src/gnc_executive/mission/coordinates.txt'
gps_coords = []
current_coord_index = 0

def load_gps_coordinates(file_path):
    """
    Loads GPS coordinates from a file and stores them in a list.
    """
    global gps_coords
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                if line.strip():  # Check if line is not empty
                    
                    latitude, longitude, altitude = map(float, line.strip().split(','))
                    gps_coords.append(GeoPointStamped(
                        header=Header(stamp=rospy.Time.now()),
                        position=GeoPoint(latitude=latitude, longitude=longitude, altitude=altitude)
                    ))
    except IOError as e:
        rospy.logerr("Could not read file: {}".format(e))

def get_next_coordinate(req):
    """
    Service handler function to return the next GPS coordinate.
    """
    global current_coord_index, gps_coords
    if current_coord_index < len(gps_coords):
        next_coordinate = gps_coords[current_coord_index]
        current_coord_index += 1
        return GetNextCoordinateResponse(next_coordinate)
    else:
        # Optionally, reset index or indicate completion
        rospy.logwarn("No more coordinates.")
        return GetNextCoordinateResponse(GeoPointStamped())  # Return an empty message

def gps_coordinate_service():
    """
    Initializes the ROS service for getting the next GPS coordinate.
    """
    rospy.Service('get_next_coordinate', GetNextCoordinate, get_next_coordinate)
    rospy.loginfo("Service get_next_coordinate ready.")

def main():
    rospy.init_node('gps_coordinate_provider')
    load_gps_coordinates(file_path)
    gps_coordinate_service()
    rospy.spin()  # Keep the node running to provide the service

if __name__ == '__main__':
    main()

