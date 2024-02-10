

#include <gnc_control/roboclaw.h>
#include <gnc_control/robot_hardware_interface.h>
#include <vector>
#include <string>
#include <math.h>

void GetYamlParameters(ros::NodeHandle*, RobotHwinSettings*, RoboclawSettings*);
bool validateSettingsAndLogErrors(RobotHwinSettings*);

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Loading wheel_hardware_interface_node");
    ros::init(argc, argv, "hardware_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO("Getting yaml parameters for wheel hardware interface settings");
    RobotHwinSettings robotSettings;
    RoboclawSettings roboclawSettings;
    GetYamlParameters(&nh, &robotSettings, &roboclawSettings);

    ROS_INFO("Initializing robot hardware interface");
    RobotHardwareInterface robotHwin(&nh, &robotSettings);

    ROS_INFO("Initializing controller manager");
    controller_manager::ControllerManager cm(&robotHwin);

    ROS_INFO("Initializing roboclaw interface");
    Roboclaw roboclaw(&roboclawSettings);
    ros::Rate rate(robotSettings.rosLoopRate);

    while (ros::ok()) {
        robotHwin.read(&roboclaw);
        cm.update(robotHwin.getTime(), robotHwin.getPeriod());
        robotHwin.write(&roboclaw);
        rate.sleep();
    }

    return 0;
}

void GetYamlParameters(ros::NodeHandle* nh, RobotHwinSettings *robotSettings, RoboclawSettings *roboclawSettings) {
    for (int i = 0; i < 3; i++) 
    {
        robotSettings->rightWheelRoboclawAddresses[i] = 0;
        robotSettings->leftWheelRoboclawAddresses[i] = 0;
        robotSettings->armRoboclawAddresses[i] = 0;
    }
    for (int i = 0; i < 6; i++)
    {
        robotSettings->armRoboclawAddresses[i] = 0;
    }

    nh->getParam("/roboclaw_settings/serial_port", roboclawSettings->serialPortAddress);
    nh->getParam("/roboclaw_settings/send_command_retries", roboclawSettings->retries);
    nh->getParam("/roboclaw_settings/encoder_timeout_ms", roboclawSettings->timeoutMs);
    nh->getParam("/roboclaw_settings/loop_frequency", roboclawSettings->loopFrequency);
    nh->getParam("/roboclaw_settings/baud_rate", roboclawSettings->baudRate);

    std::vector<std::string> rightWheelNames, leftWheelNames, armJointNames; // list of robot's joint names
    std::vector<int> leftWheelAddresses, rightWheelAddresses, armRoboclawAddresses; // each wheel's corresponding roboclaw addresses
   
    // wheel settings
    nh->getParam("/wheel_hwin_settings/debug_mode", robotSettings->debugMode);
    nh->getParam("/roboclaw_settings/send_command_retries", robotSettings->maxRetries);
    nh->getParam("/wheel_hwin_settings/ros_loop_rate", robotSettings->rosLoopRate);
    nh->getParam("/wheel_hwin_settings/left_addr", leftWheelAddresses);
    nh->getParam("/wheel_hwin_settings/right_addr", rightWheelAddresses);
    nh->getParam("/wheel_hwin_settings/left_wheel", leftWheelNames);
    nh->getParam("/wheel_hwin_settings/right_wheel", rightWheelNames);

    //arm settings
    nh->getParam("/arm_hwin_settings/joint_names", armJointNames);
    nh->getParam("/arm_hwin_settings/joint_addresses", armRoboclawAddresses);

        // copy ros_params settings into wheel settings struct
    // wheel
    std::copy(leftWheelNames.begin(), leftWheelNames.end(), robotSettings->leftWheelNames);
    std::copy(rightWheelNames.begin(), rightWheelNames.end(), robotSettings->rightWheelNames);
    std::copy(leftWheelAddresses.begin(), leftWheelAddresses.end(), robotSettings->leftWheelRoboclawAddresses);
    std::copy(rightWheelAddresses.begin(), rightWheelAddresses.end(), robotSettings->rightWheelRoboclawAddresses);

    // arm
    std::copy(armJointNames.begin(), armJointNames.end(), robotSettings->armJointNames);


    bool errorFlag = validateSettingsAndLogErrors(robotSettings);

    if (errorFlag)
        exit(EXIT_FAILURE);
}

bool validateSettingsAndLogErrors(RobotHwinSettings *robotSettings)
{
    bool errorFlag = false;
    for (int i = 0; i < 3; i++)
    {
        // error checking
        if (robotSettings->rightWheelNames[i] == "") {
            ROS_ERROR("Right joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            errorFlag = true;
        }

        if (robotSettings->rightWheelRoboclawAddresses[i] < 128 ||
             robotSettings->rightWheelRoboclawAddresses[i] > 135) {
            ROS_ERROR("Right address [%d] : Incorrect address "
                      "specified in YAML file", i);
            errorFlag = true;
        }

        if (robotSettings->leftWheelNames[i] == "") {
            ROS_ERROR("Left Joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            errorFlag = true;
        }

        if (robotSettings->leftWheelRoboclawAddresses[i] < 128 || 
            robotSettings->leftWheelRoboclawAddresses[i] > 135) {
            ROS_ERROR("Left address [%d] : Incorrect address "
                      "specified in YAML file", i);
            errorFlag = true;
        }
    }

    return errorFlag;
}
