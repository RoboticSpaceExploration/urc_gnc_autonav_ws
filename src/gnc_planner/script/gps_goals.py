#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from move_base_msgs.msg import MoveBaseActionResult

# Temporary txt file for the latitude, longitude, and altitude
file_path = './coordinates.txt'

pub = None

gps_coords = []
geo_point_stamped = []
currentCoord = 0
success = False

def populate_gps_coords(file_path, desired_list):
    global gps_coords
    try:
        with open(file_path, 'r') as file:
            content = file.read()
            lines = content.split('\n')
            for line in lines:
                if(len(line) > 1):
                    gps_coords.append(line)
        return len(gps_coords)
    except IOError as e: 
        print("Error reading file: {}".format(e))

    
def init_gps_config():
    global gps_coords
    
    no_of_coords = populate_gps_coords(file_path,gps_coords)
    print(f"Finished loading {no_of_coords} GPS coords from file")
    populate_geo_point_stamped(gps_coords)

def populate_geo_point_stamped(desired_list):
    global geo_point_stamped
    global gps_coords
    for element in gps_coords:
        spl = [float(i) for i in element.split(',')]
        geo_point  = GeoPointStamped()
        geo_point.position = GeoPoint(latitude = spl[0], longitude = spl[1], altitude = spl[2])
        geo_point.header =  Header(stamp=rospy.Time.now())
        geo_point_stamped.append(geo_point)
    print_geo_point_stamped(geo_point_stamped)
def print_geo_point_stamped(desired_list):
    for element in geo_point_stamped:
        print("Latitude: {}, Longitude: {}, Altitude: {}".format(element.position.latitude, element.position.longitude, element.position.altitude))

def check_move_base_result(result):
    return result.status.status == 3

def move_base_result_callback(msg):
    success = check_move_base_result(msg)
    print("Completed checkpoint, moving onto next one")
    global currentCoord

    if success:
        currentCoord += 1
    global pub

    msg = geo_point_stamped[currentCoord]
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    
def setup_move_base_result_listener():
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

def start_node():
    global pub

    rospy.init_node('gps_goals_node')
    rate = rospy.Rate(100)
    pub = rospy.Publisher('/gps_goal', GeoPointStamped, queue_size=10)
    rospy.loginfo("GPS Publisher Node Started")
    
    init_gps_config()
    setup_move_base_result_listener()

    msg = geo_point_stamped[0]
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

    print(currentCoord)
    #  Start main node loop
    #while not rospy.is_shutdown():
    #    msg = geo_point_stamped[currentCoord]
    #    msg.header.stamp = rospy.Time.now()
    #    pub.publish(msg)
    #    rate.sleep()
    rospy.spin() 

if __name__ == '__main__':
    start_node()

