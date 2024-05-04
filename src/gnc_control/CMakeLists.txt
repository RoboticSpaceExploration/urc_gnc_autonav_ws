cmake_minimum_required(VERSION 3.0.2)
project(gnc_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  controller_manager_msgs
  gazebo_ros
  joint_state_controller
  urdf
  std_msgs
  roscpp
  hardware_interface
  tf
)

add_message_files(
  FILES
  odrive_msg.msg
  odrive_ctrl.msg
)

add_service_files(
  FILES
  CalibrationService.srv
  EncoderCount.srv
  SliderPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(usb-1.0_LIBRARIES usb-1.0)
set(jsoncpp_LIBRARIES jsoncpp)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    controller_manager 
    roscpp 
    message_runtime
    std_msgs
    hardware_interface
  DEPENDS 
    ${rt_LIBRARIES} 
    ${pthread_LIBRARIES} 
    ${usb-1.0_LIBRARIES} 
    ${jsoncpp_LIBRARIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
)

# add_executable(single_joint_hardware_interface src/robot_hardware_interface.cpp)
# # add_dependencies(robot_hardware_interface ${catkin_EXPORTED_TARGETS})
# target_link_libraries(single_joint_hardware_interface ${catkin_LIBRARIES})

add_executable(drive_node 
  src/drive_node.cpp
  src/drive_hwin.cpp
  src/odrive_utils.cpp
  src/odrive_endpoint.cpp
)
target_link_libraries(drive_node 
  ${usb-1.0_LIBRARIES} 
  ${catkin_LIBRARIES} 
  ${jsoncpp_LIBRARIES}
)
add_dependencies(drive_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(control 
  src/control.cpp 
  src/robot_hardware_interface.cpp 
  src/roboclaw.cpp
)
target_link_libraries(control ${catkin_LIBRARIES})
link_directories(${catkin_LIBRARY_DIRS})

#add_executable(wheel_hardware_interface src/wheel_hardware_interface.cpp src/roboclaw.cpp)
#target_link_libraries(wheel_hardware_interface ${catkin_LIBRARIES})
#link_directories(${catkin_LIBRARY_DIRS})
#
#add_executable(arm_hardware_interface src/arm_hardware_interface.cpp src/roboclaw.cpp)
#target_link_libraries(arm_hardware_interface ${catkin_LIBRARIES})
#link_directories(${catkin_LIBRARY_DIRS})
#

#############
## Install ##
#############

install(TARGETS drive_node 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS drive_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/arm_teleop.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

