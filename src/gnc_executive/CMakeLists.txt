cmake_minimum_required(VERSION 3.0.2)
project(gnc_executive)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
		rospy
		geographic_msgs
		message_generation)

catkin_python_setup()

add_service_files(
  FILES
  GetNextCoordinate.srv
  UpdateCoordinates.srv
  GPS2UTM.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geographic_msgs  # Or other packages containing msgs
 )

catkin_package()
