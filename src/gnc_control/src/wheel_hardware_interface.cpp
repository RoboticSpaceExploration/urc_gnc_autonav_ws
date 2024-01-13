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

#include <gnc_control/roboclaw.h>
#include <gnc_control/wheel_hardware_interface.h>
#include <vector>
#include <string>
#include <math.h>
#include <cstdlib>


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
    hardware_interface::JointStateHandle wheelRightFront( wheelSettings->rightWheelNames[0], &pos[0], &vel[0], &eff[0]);
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

    sendCommandToWheels(rb);
}

void WheelHardwareInterface::readFromWheels(Roboclaw *rb)
{
    if (wheelSettings->debugMode) {
        rb->GetVelocityFromWheels(vel);

        ROS_INFO_STREAM("READING JOINT STATES FROM MOTOR ENCODERS");
        printDebugInfo("VEL FROM", vel);
    }
}

void WheelHardwareInterface::sendCommandToWheels(Roboclaw* rb)
{
    // convert cmd_vel to a usable command between 0-127
    scaleCommands();

    // prevent zero velocity spamming from ros_control
    if (zeroCmdVelCount <= wheelSettings->maxRetries) {
        // if positive, move motors forward. if negative, move backwards
        if (cmd[0] >= 0)  // right_front
            rb->ForwardM1(0x80, cmdToSend[0]);
        else
            rb->BackwardM1(0x80, cmdToSend[0]);

        if (cmd[1] >= 0)  // right_middle
            rb->ForwardM1(0x81, cmdToSend[1]);
        else
            rb->BackwardM1(0x81, cmdToSend[1]);

        if (cmd[2] >= 0)  // right_back
            rb->ForwardM1(0x82, cmdToSend[2]);
        else
            rb->BackwardM1(0x82, cmdToSend[2]);

        if (cmd[3] >= 0)  // left_front
            rb->ForwardM2(0x80, cmdToSend[3]);
        else
            rb->BackwardM2(0x80, cmdToSend[3]);

        if (cmd[4] >= 0)  // left_middle
            rb->ForwardM2(0x81, cmdToSend[4]);
        else
            rb->BackwardM2(0x81, cmdToSend[4]);

        if (cmd[5] >= 0)  // left_back
            rb->ForwardM2(0x82, cmdToSend[5]);
        else
            rb->BackwardM2(0x82, cmdToSend[5]);
    }

    // if any of the cmd_vel are zero, increment counter

    if (cmd[0] == 0 || cmd[1] == 0 || cmd[2] == 0 ||
        cmd[3] == 0 || cmd[4] == 0 || cmd[5] == 0) {
        zeroCmdVelCount++;
    } else {
        zeroCmdVelCount = 0;  // reset counter
        cmd[0] = cmd[1] = cmd[2] = cmd[3] = cmd[4] = cmd[5] = 0;
    }
}

void WheelHardwareInterface::scaleCommands()
{
    // TODO scaled the commands from 0-126 to send to roboclaw
    for (int i = 0; i < 6; i++)
    {
        double res = (fabs(cmd[i]) / 16.667) * wheelSettings->maxEffortValue;

        if (res >= wheelSettings->maxEffortValue)
            cmdToSend[i] = wheelSettings->maxEffortValue;
        else
            cmdToSend[i] = (uint8_t) res;
    }
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
