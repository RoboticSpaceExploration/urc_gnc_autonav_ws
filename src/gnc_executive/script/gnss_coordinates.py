#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Header
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from gnc_executive.srv import GetNextCoordinate, GetNextCoordinateResponse, UpdateCoordinates, UpdateCoordinatesResponse, GetNextNoIncrement, GetNextNoIncrementResponse, SetNextWaypoint, SetNextWaypointResponse

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
                    gps_coords.append([GeoPointStamped(
                        header=Header(stamp=rospy.Time.now()),
                        position=GeoPoint(latitude=latitude, longitude=longitude, altitude=altitude)
                    ),0])
    except IOError as e:
        rospy.logerr(f"Could not read file: {e}")

def update_gps_coordinates(req):
    """
    Service handler function to update GPS coordinates including waypoint types.
    """
    global gps_coords
    try:
        new_coords = []
        for coord in req.coordinates:
            latitude, longitude, altitude, wypt_type = map(float, coord.split(','))
            new_coords.append([ GeoPointStamped(
                    header=Header(stamp=rospy.Time.now()),
                    position=GeoPoint(latitude=latitude, longitude=longitude, altitude=altitude)
                ),
                int(wypt_type)],  # Ensure waypoint type is an integer
                
                
            )
        gps_coords = new_coords
        current_coord_index = 0  # Reset index after update
        rospy.loginfo("Received new coordinates!")
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
        next_coordinate = gps_coords[current_coord_index][0]
        wypt_type = gps_coords[current_coord_index][1]
        current_coord_index += 1
        return GetNextCoordinateResponse(next_coordinate,wypt_type)
    else:
        rospy.logwarn("No more coordinates.")
        return GetNextCoordinateResponse(GeoPointStamped(),-1)

def get_next_noinc(req):
    """
    Service handler function to peek at the next GPS coordinate without incrementing the index.
    """
    global current_coord_index, gps_coords
    if current_coord_index < len(gps_coords):
        next_coordinate = gps_coords[current_coord_index - 1][0] 
        wypt_type = gps_coords[current_coord_index - 1][1] 
        return GetNextNoIncrementResponse(current_coord_index,next_coordinate,wypt_type)
    else:
        rospy.logwarn("No more coordinates.")
        return GetNextNoIncrementResponse(GeoPointStamped())

def set_next_waypoint(req):
    """
    Service to set the current waypoint index.
    """
    global current_coord_index
    if 0 <= req.index < len(gps_coords):
        current_coord_index = req.index
        rospy.loginfo(f"Waypoint index set to {current_coord_index}")
        return SetNextWaypointResponse(True)
    else:
        rospy.logerr("Invalid waypoint index.")
        return SetNextWaypointResponse(False)

def gps_coordinate_service():
    """
    Initializes the ROS services for GPS coordinates.
    """
    rospy.Service('get_next_coordinate', GetNextCoordinate, get_next_coordinate)
    rospy.Service('get_next_noinc', GetNextNoIncrement, get_next_noinc)
    rospy.Service('set_next_waypoint', SetNextWaypoint, set_next_waypoint)
    rospy.Service('update_coordinates', UpdateCoordinates, update_gps_coordinates)
    rospy.loginfo("Services ready.")

def main():
    rospy.init_node('gps_coordinate_provider')
    load_gps_coordinates(file_path)
    gps_coordinate_service()
    rospy.spin()

if __name__ == '__main__':
    main()
