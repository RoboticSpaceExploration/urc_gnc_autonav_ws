#include <gnc_control/drive_hwin.h>
#include <gnc_control/odrive.hpp>
#include <algorithm>
#include <string>
#include <vector>

int GetYamlParameters(ros::NodeHandle &nh, DriveHwinSettings &driveSettings)
{
    bool success = true;
    std::vector<std::string> leftWheelNames, rightWheelNames;
    std::vector<int> leftWheelAxis, rightWheelAxis;

    // read all settings params
    nh.getParam("/drive_hwin_settings/left_wheel_names", leftWheelNames);
    nh.getParam("/drive_hwin_settings/right_wheel_names", rightWheelNames);
    nh.getParam("/drive_hwin_settings/left_wheel_axis", leftWheelAxis);
    nh.getParam("/drive_hwin_settings/left_wheel_axis", rightWheelAxis);
    nh.getParam("/drive_hwin_settings/ros_loop_rate", driveSettings.rosLoopRate);
    nh.getParam("/drive_hwin_settings/debug_mode", driveSettings.debugMode);
    nh.getParam("/drive_hwin_settings/encoder_ticks_per_revolution", driveSettings.encoderTicksPerRevolution);
    
    // error checking names and axis
    if (leftWheelNames.size() != 2)
    {
        ROS_ERROR("There should be two left wheel names. Found %lu names", leftWheelNames.size());
        success = false;
    }
    if (rightWheelNames.size() != 2)
    {
        ROS_ERROR("There should be two right wheel names. Found %lu names", rightWheelNames.size());
        success = false;
    }
    if (leftWheelAxis.size() != 2)
    {
        ROS_ERROR("There should be two left wheel axis. Found %lu axis", leftWheelAxis.size());
        success = false;
    }
    if (rightWheelAxis.size() != 2)
    {
        ROS_ERROR("There should be two right wheel axis. Found %lu axis", rightWheelAxis.size());
        success = false;
    }
    if (!success)
        return -1;

    // copy vectors to arrays
    for (int i = 0; i < leftWheelNames.size(); i++)
        driveSettings.leftWheelNames[i] = leftWheelNames[i];
    for (int i = 0; i < rightWheelNames.size(); i++)
        driveSettings.rightWheelNames[i] = rightWheelNames[i];
    for (int i = 0; i < leftWheelAxis.size(); i++)
        driveSettings.leftWheelAxis[i] = leftWheelAxis[i];
    for (int i = 0; i < rightWheelAxis.size(); i++)
        driveSettings.rightWheelAxis[i] = rightWheelAxis[i];

    ROS_INFO_STREAM("left name 0: " << driveSettings.leftWheelNames[0]);
    ROS_INFO_STREAM("left name 1: " << driveSettings.leftWheelNames[1]);
    ROS_INFO_STREAM("right name 0: " << driveSettings.rightWheelNames[0]);
    ROS_INFO_STREAM("right name 1: " << driveSettings.rightWheelNames[1]);

    return 0;
}

/**
 *
 * Node main function
 *
 */
int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "drive_node"); // Initializes Node Name
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(3); // needed for controllers to subscribe to topics
    spinner.start();

    // read settings
    ROS_INFO("Reading and applying configuration settings");
    DriveHwinSettings driveSettings;
    if (GetYamlParameters(nh, driveSettings))
    {
        ROS_ERROR("Error reading rosparams");
    }

    ROS_INFO("Initializing DriveHwin");
    DriveHwin driveHwin(&nh, driveSettings);
    controller_manager::ControllerManager cm(&driveHwin);

    ros::Rate r(driveSettings.rosLoopRate);
    ROS_INFO("Done initializing. Entered control loop");
    while (ros::ok())
    {
        // read, update, write control loop
        driveHwin.read();
        cm.update(driveHwin.getTime(), driveHwin.getPeriod());
        driveHwin.write();
        r.sleep();
    }

    return 0;
}
