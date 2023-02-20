/* MIT License
Copyright (c) [2022] [VIP Team RoSE]
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#include <gnc_control/wheel_hardware_interface.h>
#include <vector>
#include <string>

void GetYamlParameters(ros::NodeHandle*, WheelHwinSettings*, RoboclawSettings*);
bool validateSettingsAndLogErrors(WheelHwinSettings*);

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Loading wheel_hardware_interface_node");
    ros::init(argc, argv, "hardware_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO("Getting yaml parameters for wheel hardware interface settings");
    WheelHwinSettings wheelSettings;
    RoboclawSettings roboclawSettings;
    GetYamlParameters(&nh, &wheelSettings, &roboclawSettings);

    ROS_INFO("Initializing wheel hardware interface");
    WheelHardwareInterface wheelHwin(&nh, &wheelSettings);

    ROS_INFO("Initializing controller manager");
    controller_manager::ControllerManager cm(&wheelHwin);

    ROS_INFO("Initializing roboclaw interface");
    Roboclaw roboclaw(&roboclawSettings);
    ros::Rate rate(wheelSettings.rosLoopRate);

    while (ros::ok()) {
        wheelHwin.readFromWheels(&roboclaw);
        cm.update(wheelHwin.get_time(), wheelHwin.get_period());
        wheelHwin.writeToWheels(&roboclaw);
        rate.sleep();
    }

    return 0;
}

void GetYamlParameters(ros::NodeHandle* nh, WheelHwinSettings *wheelSettings, RoboclawSettings *roboclawSettings) {
    for (int i = 0; i < 3; i++) {
        wheelSettings->rightWheelRoboclawAddresses[i] = 0;
        wheelSettings->leftWheelRoboclawAddresses[i] = 0;
    }

    nh->getParam("/roboclaw_settings/serial_port", roboclawSettings->serialPortAddress);
    nh->getParam("/roboclaw_settings/send_command_retries", roboclawSettings->retries);
    nh->getParam("/roboclaw_settings/encoder_timeout_ms", roboclawSettings->timeoutMs);
    nh->getParam("/roboclaw_settings/loop_frequency", roboclawSettings->loopFrequency);
    nh->getParam("/roboclaw_settings/baud_rate", roboclawSettings->baudRate);

    std::vector<std::string> rightWheelNames, leftWheelNames; // list of wheel joint names
    std::vector<int> leftWheelAddresses, rightWheelAddresses; // each wheel's corresponding roboclaw addresses
    nh->getParam("/wheel_hwin_settings/debug_mode", wheelSettings->debugMode);
    nh->getParam("/wheel_hwin_settings/ros_loop_rate", wheelSettings->rosLoopRate);
    nh->getParam("/wheel_hwin_settings/left_addr", leftWheelAddresses);
    nh->getParam("/wheel_hwin_settings/right_addr", rightWheelAddresses);
    nh->getParam("/wheel_hwin_settings/left_wheel", leftWheelNames);
    nh->getParam("/wheel_hwin_settings/right_wheel", rightWheelNames);

    for (int i = 0; i < 3; i++) {
        // copy ros_params settings into wheel settings struct
        std::copy(leftWheelNames.begin(), leftWheelNames.end(), wheelSettings->leftWheelNames);
        std::copy(rightWheelNames.begin(), rightWheelNames.end(), wheelSettings->rightWheelNames);
        std::copy(leftWheelAddresses.begin(), leftWheelAddresses.end(), wheelSettings->leftWheelRoboclawAddresses);
        std::copy(rightWheelAddresses.begin(), rightWheelAddresses.end(), wheelSettings->rightWheelRoboclawAddresses);

    }

    bool errorFlag = validateSettingsAndLogErrors(wheelSettings);

    if (errorFlag)
        exit(EXIT_FAILURE);
}

bool validateSettingsAndLogErrors(WheelHwinSettings *wheelSettings)
{
    bool errorFlag = false;
    for (int i = 0; i < 3; i++)
    {
        // error checking
        if (wheelSettings->rightWheelNames[i] == "") {
            ROS_ERROR("Right joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            errorFlag = true;
        }

        if (wheelSettings->rightWheelRoboclawAddresses[i] < 128 ||
             wheelSettings->rightWheelRoboclawAddresses[i] > 135) {
            ROS_ERROR("Right address [%d] : Incorrect address "
                      "specified in YAML file", i);
            errorFlag = true;
        }

        if (wheelSettings->leftWheelNames[i] == "") {
            ROS_ERROR("Left Joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            errorFlag = true;
        }

        if (wheelSettings->leftWheelRoboclawAddresses[i] < 128 || 
            wheelSettings->leftWheelRoboclawAddresses[i] > 135) {
            ROS_ERROR("Left address [%d] : Incorrect address "
                      "specified in YAML file", i);
            errorFlag = true;
        }
    }

    return errorFlag;
}

WheelHardwareInterface::WheelHardwareInterface(ros::NodeHandle* nh, WheelHwinSettings* wheelSettings)
{
    this->wheelSettings = wheelSettings;
    ROS_INFO("Registering ros_control joint interfaces");
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    for (int i = 0; i < 6; i++)
        cmd[i] = vel[i] = pos[i] = eff[i] = 0;
}

WheelHardwareInterface::~WheelHardwareInterface() { }

void WheelHardwareInterface::registerStateHandlers()
{
    ROS_INFO("Registering Joint State Interface");
    ROS_INFO_STREAM(wheelSettings->rightWheelNames[0]);
    hardware_interface::JointStateHandle wheelRightFront(
        wheelSettings->rightWheelNames[0], &pos[0], &vel[0], &eff[0]);
    jointStateInterface.registerHandle(wheelRightFront);

    hardware_interface::JointStateHandle wheelRightMiddle(
        wheelSettings->rightWheelNames[1], &pos[1], &vel[1], &eff[1]);
    jointStateInterface.registerHandle(wheelRightMiddle);

    hardware_interface::JointStateHandle wheelRightBack(
        wheelSettings->rightWheelNames[2], &pos[2], &vel[2], &eff[2]);
    jointStateInterface.registerHandle(wheelRightBack);

    hardware_interface::JointStateHandle wheelLeftFront(
        wheelSettings->leftWheelNames[0], &pos[3], &vel[3], &eff[3]);
    jointStateInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointStateHandle wheelLeftMiddle(
        wheelSettings->leftWheelNames[1], &pos[4], &vel[4], &eff[4]);
    jointStateInterface.registerHandle(wheelLeftMiddle);

    hardware_interface::JointStateHandle wheelLeftBack(
        wheelSettings->leftWheelNames[2], &pos[5], &vel[5], &eff[5]);
    jointStateInterface.registerHandle(wheelLeftBack);

    registerInterface(&jointStateInterface);
}
void WheelHardwareInterface::registerJointVelocityHandlers()
{
    ROS_INFO("Registering Velocity Joint Interface");
    hardware_interface::JointHandle wheelRightFront(
        jointStateInterface.getHandle(wheelSettings->rightWheelNames[0]), &cmd[0]);
    velocityJointInterface.registerHandle(wheelRightFront);

    hardware_interface::JointHandle wheelRightMiddle(
        jointStateInterface.getHandle(wheelSettings->rightWheelNames[1]), &cmd[1]);
    velocityJointInterface.registerHandle(wheelRightMiddle);

    hardware_interface::JointHandle wheelRightBack(
        jointStateInterface.getHandle(wheelSettings->rightWheelNames[2]), &cmd[2]);
    velocityJointInterface.registerHandle(wheelRightBack);

    hardware_interface::JointHandle wheelLeftFront(
        jointStateInterface.getHandle(wheelSettings->leftWheelNames[0]), &cmd[3]);
    velocityJointInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointHandle wheelLeftMiddle(
        jointStateInterface.getHandle(wheelSettings->leftWheelNames[1]), &cmd[4]);
    velocityJointInterface.registerHandle(wheelLeftMiddle);

    hardware_interface::JointHandle wheelLeftBack(
        jointStateInterface.getHandle(wheelSettings->leftWheelNames[2]), &cmd[5]);
    velocityJointInterface.registerHandle(wheelLeftBack);

    registerInterface(&velocityJointInterface);
}

void WheelHardwareInterface::writeToWheels(Roboclaw *rb)
{
    if (wheelSettings->debugMode) {
        ROS_INFO_STREAM("READING JOINT STATES FROM ROS");
        printDebugInfo("SENDING CMD_VEL TO", cmd);
    }

    rb->SendCommandToWheels(cmd);
}

void WheelHardwareInterface::readFromWheels(Roboclaw *rb)
{
    if (wheelSettings->debugMode) {
        rb->GetVelocityFromWheels(vel);

        ROS_INFO_STREAM("READING JOINT STATES FROM MOTOR ENCODERS");
        printDebugInfo("VEL FROM", vel);
    }
}

void WheelHardwareInterface::sendCommandToWheels(Roboclaw* roboclaw)
{

}

void WheelHardwareInterface::scaleCommands()
{
    // TODO scaled the commands from 0-126 to send to roboclaw
}

void WheelHardwareInterface::getVelocityFromEncoders()
{
    // TODO use convertPulsesToRadians for distance conversion
    // velocity = distance / time
}

double WheelHardwareInterface::convertPulsesToRadians(double cmd)
{
    // TODO conversion rate from motor encoder value to distance (circumference of wheel)
    return 0;
}


ros::Time WheelHardwareInterface::get_time()
{
    return ros::Time::now();
}

ros::Duration WheelHardwareInterface::get_period()
{
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time =
            ros::Duration(current_time.tv_sec - last_time.tv_sec
            + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    return elapsed_time;
}

void WheelHardwareInterface::printDebugInfo(std::string name, double* data) {
    ROS_INFO_STREAM(name << " RIGHT_FRONT_WHEEL_JOINT "  << data[0]);
    ROS_INFO_STREAM(name << " RIGHT_MIDDLE_WHEEL_JOINT " << data[1]);
    ROS_INFO_STREAM(name << " RIGHT_BACK_WHEEL_JOINT "   << data[2]);
    ROS_INFO_STREAM(name << " LEFT_FRONT_WHEEL_JOINT "   << data[3]);
    ROS_INFO_STREAM(name << " LEFT_MIDDLE_WHEEL_JOINT "  << data[4]);
    ROS_INFO_STREAM(name << " LEFT_BACK_WHEEL_JOINT "    << data[5]);
}
