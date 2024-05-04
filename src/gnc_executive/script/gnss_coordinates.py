#!/bin/bash python3
import rospy
import numpy as np
from std_msgs.msg import Header
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from gnc_executive.srv import GetNextCoordinate, GetNextCoordinateResponse, UpdateCoordinates, UpdateCoordinatesResponse

# Global variables
file_path = '/home/roselab/Desktop/urc_gnc_autonav_ws/src/gnc_executive/mission/coordinates.txt'
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
            gps_coords.clear()  # Clear existing coordinates
            for line in lines:
                if line.strip():
                    latitude, longitude, altitude = map(np.float64, line.strip().split(','))
                    gps_coords.append(GeoPointStamped(
                        header=Header(stamp=rospy.Time.now()),
                        position=GeoPoint(latitude=latitude, longitude=longitude, altitude=altitude)
                    ))
    except IOError as e:
        rospy.logerr(f"Could not read file: {e}")

def update_gps_coordinates(req):
    """
    Service handler function to update GPS coordinates.
    """
    global gps_coords
    try:
        new_coords = []
        for coord in req.coordinates:
            latitude, longitude, altitude = map(float, coord.split(','))
            new_coords.append(GeoPointStamped(
                header=Header(stamp=rospy.Time.now()),
                position=GeoPoint(latitude=latitude, longitude=longitude, altitude=altitude)
            ))
        gps_coords = new_coords
        current_coord_index = 0  # Reset index after update
        return UpdateCoordinatesResponse(True)
    except Exception as e:
        rospy.logerr(f"Error updating coordinates: {e}")
        return UpdateCoordinatesResponse(False)

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
        rospy.logwarn("No more coordinates.")
        return GetNextCoordinateResponse(GeoPointStamped())

def gps_coordinate_service():
    """
    Initializes the ROS services for GPS coordinates.
    """
    rospy.Service('get_next_coordinate', GetNextCoordinate, get_next_coordinate)
    rospy.Service('update_coordinates', UpdateCoordinates, update_gps_coordinates)
    rospy.loginfo("Services get_next_coordinate and update_coordinates ready.")

def main():
    rospy.init_node('gps_coordinate_provider')
    load_gps_coordinates(file_path)
    gps_coordinate_service()
    rospy.spin()  # Keep the node running to provide the services

if __name__ == '__main__':
    main()
