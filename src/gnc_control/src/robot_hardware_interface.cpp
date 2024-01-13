
#include <gnc_control/roboclaw.h>
#include <gnc_control/robot_hardware_interface.h>
#include <vector>
#include <string>
#include <math.h>
#include <cstdlib>

RobotHardwareInterface::RobotHardwareInterface(ros::NodeHandle* nh, RobotHwinSettings* robotSettings)
{
    this->robotSettings = robotSettings;
	
    ROS_INFO("Registering ros_control joint interfaces and handlers");
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &lastTime);

    for (int i = 0; i < 11; i++)
        cmd[i] = vel[i] = pos[i] = eff[i] = 0;
}

RobotHardwareInterface::~RobotHardwareInterface() { }

void RobotHardwareInterface::registerStateHandlers()
{
    ROS_INFO("Registering Joint State Interface");

    // wheel state handlers
    hardware_interface::JointStateHandle wheelRightFront( 
        robotSettings->rightWheelNames[0], &pos[0], &vel[0], &eff[0]);
    jointStateInterface.registerHandle(wheelRightFront);

    hardware_interface::JointStateHandle wheelRightMiddle(
        robotSettings->rightWheelNames[1], &pos[1], &vel[1], &eff[1]);
    jointStateInterface.registerHandle(wheelRightMiddle);

    hardware_interface::JointStateHandle wheelRightBack(
        robotSettings->rightWheelNames[2], &pos[2], &vel[2], &eff[2]);
    jointStateInterface.registerHandle(wheelRightBack);

    hardware_interface::JointStateHandle wheelLeftFront(
        robotSettings->leftWheelNames[0], &pos[3], &vel[3], &eff[3]);
    jointStateInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointStateHandle wheelLeftMiddle(
        robotSettings->leftWheelNames[1], &pos[4], &vel[4], &eff[4]);
    jointStateInterface.registerHandle(wheelLeftMiddle);

    hardware_interface::JointStateHandle wheelLeftBack(
        robotSettings->leftWheelNames[2], &pos[5], &vel[5], &eff[5]);
    jointStateInterface.registerHandle(wheelLeftBack);

    // Arm state handlers
    hardware_interface::JointStateHandle base_to_shoulder_joint(
        robotSettings->armJointNames[0], &pos[6], &vel[6], &eff[6]);
    jointStateInterface.registerHandle(base_to_shoulder_joint);

    hardware_interface::JointStateHandle shoulder_to_arm_joint(
        robotSettings->armJointNames[1], &pos[7], &vel[7], &eff[7]);
    jointStateInterface.registerHandle(shoulder_to_arm_joint);

    hardware_interface::JointStateHandle arm_to_yzwrist_joint(
        robotSettings->armJointNames[2], &pos[8], &vel[8], &eff[8]);
    jointStateInterface.registerHandle(arm_to_yzwrist_joint);

    hardware_interface::JointStateHandle yzwrist_to_xywrist_joint(
        robotSettings->armJointNames[3], &pos[9], &vel[9], &eff[9]);
    jointStateInterface.registerHandle(yzwrist_to_xywrist_joint);

    hardware_interface::JointStateHandle xywrist_to_xzwrist_joint(
        robotSettings->armJointNames[4], &pos[10], &vel[10], &eff[10]);
    jointStateInterface.registerHandle(xywrist_to_xzwrist_joint);

    hardware_interface::JointStateHandle xzwrist_to_grip_joint(
        robotSettings->armJointNames[5], &pos[11], &vel[11], &eff[11]);
    jointStateInterface.registerHandle(xzwrist_to_grip_joint);

    registerInterface(&jointStateInterface);
}

void RobotHardwareInterface::registerJointVelocityHandlers()
{
    ROS_INFO("Registering Velocity Joint Interface");

    hardware_interface::JointHandle wheelRightFront(
        jointStateInterface.getHandle(robotSettings->rightWheelNames[0]), &cmd[0]);
    velocityJointInterface.registerHandle(wheelRightFront);

    hardware_interface::JointHandle wheelRightMiddle(
        jointStateInterface.getHandle(robotSettings->rightWheelNames[1]), &cmd[1]);
    velocityJointInterface.registerHandle(wheelRightMiddle);

    hardware_interface::JointHandle wheelRightBack(
        jointStateInterface.getHandle(robotSettings->rightWheelNames[2]), &cmd[2]);
    velocityJointInterface.registerHandle(wheelRightBack);

    hardware_interface::JointHandle wheelLeftFront(
        jointStateInterface.getHandle(robotSettings->leftWheelNames[0]), &cmd[3]);
    velocityJointInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointHandle wheelLeftMiddle(
        jointStateInterface.getHandle(robotSettings->leftWheelNames[1]), &cmd[4]);
    velocityJointInterface.registerHandle(wheelLeftMiddle);

    hardware_interface::JointHandle wheelLeftBack(
        jointStateInterface.getHandle(robotSettings->leftWheelNames[2]), &cmd[5]);
    velocityJointInterface.registerHandle(wheelLeftBack);

    // arm joint velocity command handlers
    hardware_interface::JointHandle base_to_shoulder_joint(
        jointStateInterface.getHandle(robotSettings->armJointNames[0]), &cmd[6]);
    velocityJointInterface.registerHandle(base_to_shoulder_joint);

    hardware_interface::JointHandle shoulder_to_arm_joint(
        jointStateInterface.getHandle(robotSettings->armJointNames[1]), &cmd[7]);
    velocityJointInterface.registerHandle(shoulder_to_arm_joint);

    hardware_interface::JointHandle arm_to_yzwrist_joint(
        jointStateInterface.getHandle(robotSettings->armJointNames[2]), &cmd[8]);
    velocityJointInterface.registerHandle(arm_to_yzwrist_joint);

    hardware_interface::JointHandle yzwrist_to_xywrist_joint(
        jointStateInterface.getHandle(robotSettings->armJointNames[3]), &cmd[9]);
    velocityJointInterface.registerHandle(yzwrist_to_xywrist_joint);

    hardware_interface::JointHandle xywrist_to_xzwrist_joint(
        jointStateInterface.getHandle(robotSettings->armJointNames[4]), &cmd[10]);
    velocityJointInterface.registerHandle(xywrist_to_xzwrist_joint);

    hardware_interface::JointHandle xzwrist_to_grip_joint(
        jointStateInterface.getHandle(robotSettings->armJointNames[5]), &cmd[11]);
    velocityJointInterface.registerHandle(xzwrist_to_grip_joint);

    registerInterface(&velocityJointInterface);
}

void RobotHardwareInterface::write(Roboclaw* rb)
{
    sendCommandToWheels(rb);
    // sendCommandToArm(rb);
}

void RobotHardwareInterface::sendCommandToWheels(Roboclaw* rb)
{
    // convert cmd_vel to a usable command between 0-127
    scaleWheelCommands();

    // prevent zero velocity spamming from ros_control
    if (zeroCmdVelCount <= robotSettings->maxRetries) {
        // if positive, move motors forward. if negative, move backwards
        if (cmd[0] >= 0)  // right_front
            rb->ForwardM1(robotSettings->rightWheelRoboclawAddresses[0], cmdToSend[0]);
        else
            rb->BackwardM1(robotSettings->rightWheelRoboclawAddresses[0], cmdToSend[0]);

        if (cmd[1] >= 0)  // right_middle
            rb->ForwardM1(robotSettings->rightWheelRoboclawAddresses[1], cmdToSend[1]);
        else
            rb->BackwardM1(robotSettings->rightWheelRoboclawAddresses[1], cmdToSend[1]);

        if (cmd[2] >= 0)  // right_back
            rb->ForwardM1(robotSettings->rightWheelRoboclawAddresses[2], cmdToSend[2]);
        else
            rb->BackwardM1(robotSettings->rightWheelRoboclawAddresses[2], cmdToSend[2]);

        if (cmd[3] >= 0)  // left_front
            rb->ForwardM1(robotSettings->leftWheelRoboclawAddresses[0], cmdToSend[3]);
        else
            rb->BackwardM1(robotSettings->leftWheelRoboclawAddresses[0], cmdToSend[3]);

        if (cmd[4] >= 0)  // left_middle
            rb->ForwardM1(robotSettings->leftWheelRoboclawAddresses[1], cmdToSend[4]);
        else
            rb->BackwardM1(robotSettings->leftWheelRoboclawAddresses[1], cmdToSend[4]);

        if (cmd[5] >= 0)  // left_back
            rb->ForwardM1(robotSettings->leftWheelRoboclawAddresses[2], cmdToSend[5]);
        else
            rb->BackwardM1(robotSettings->leftWheelRoboclawAddresses[2], cmdToSend[5]);
    }

    // if any of the cmd_vel are zero, increment counter

    cmd[0] = cmd[1] = cmd[2] = cmd[3] = cmd[4] = cmd[5] = 0;
}

void RobotHardwareInterface::sendCommandToArm(Roboclaw* roboclaw)
{
    uint8_t cmd_send[12];

    for (int i = 6; i < 12; i++)
    {
        cmd_send[i] = fabs(cmd[i]); // need to send positive values to roboclaws
    }

    scaleArmCommands(cmd_send);
    
    // Linear actuators (should be roboclaw address 131 or 0x83)
    if (cmd[6] > 0)
    {
        roboclaw->ForwardM1(0x83, cmd_send[6]);
        roboclaw->ForwardM2(0x83, cmd_send[6]);
    }
    else if (cmd[6] < 0)
    {
        roboclaw->BackwardM1(0x83, cmd_send[6]);
        roboclaw->BackwardM2(0x83, cmd_send[6]);
    }
    else
    {
        roboclaw->ForwardM1(0x83, 0);
        roboclaw->ForwardM2(0x83, 0);
    }

    // other motors
    if (cmd[7] >= 0)  // arm base (should be roboclaw address 132 or 0x84)
    {
        roboclaw->ForwardM1(0x84, cmd_send[7]);
        roboclaw->ForwardM2(0x84, cmd_send[7]);
    }
    else
    {
        roboclaw->BackwardM1(0x84, cmd_send[7]);
        roboclaw->BackwardM2(0x84, cmd_send[7]);
    }

    if (cmd[8] >= 0) // forearm (should be roboclaw address 134 or 0x86)
        roboclaw->ForwardM1(0x86, cmd_send[8]);
    else
        roboclaw->BackwardM1(0x86, cmd_send[8]);

    if (cmd[9] >= 0)  // end effector and wrist (should be roboclaw address 133 or 0x85)
        roboclaw->ForwardM2(0x86, cmd_send[9]);
    else
        roboclaw->BackwardM2(0x86, cmd_send[9]);

    if (cmd[10] >= 0)  // end effector and wrist
        roboclaw->ForwardM1(0x85, cmd_send[10]);
    else
        roboclaw->BackwardM1(0x85, cmd_send[10]);


    if (cmd[11] >= 0)  // left_middle not used rn
         roboclaw->ForwardM2(0x85,  cmd_send[11]);
    else
         roboclaw->BackwardM2(0x85, cmd_send[11]);
    
    for (int i = 6; i < 12; i++)
    {
        cmd[i] = 0;
    }
}

void RobotHardwareInterface::read(Roboclaw* rb)
{
    // not implemented
    // int32_t position = rb->ReadEncoderPositionM1(0x80);
    // ROS_INFO_STREAM("Reading from roboclaw: " << position);
}

void RobotHardwareInterface::scaleWheelCommands()
{
    // TODO scaled the commands from 0-126 to send to roboclaw
    for (int i = 0; i < 6; i++)
    {
        double res = (fabs(cmd[i]) / 16.667) * robotSettings->maxEffortValue;

        if (res >= robotSettings->maxEffortValue)
            cmdToSend[i] = robotSettings->maxEffortValue;
        else
            cmdToSend[i] = (uint8_t) res;
    }
}

void RobotHardwareInterface::scaleArmCommands(uint8_t *cmd_send)
{
    int maxVal = 126;
    for (int i = 6; i < 12; i++)
    {
        cmd_send[i] *= maxVal;
        if (cmd_send[i] >= maxVal)
            cmd_send[i] = maxVal;
	else if (cmd_send[i] < 0)
	    cmd_send[i] = 0;
    }
}

void RobotHardwareInterface::getVelocityFromWheels()
{
    // TODO use convertPulsesToRadians for distance conversion
    // velocity = distance / time
}

double RobotHardwareInterface::convertPulsesToRadians(double cmd)
{
    // TODO conversion rate from motor encoder value to distance (circumference of wheel)
    return 0;
}

ros::Time RobotHardwareInterface::getTime()
{
    return ros::Time::now();
}

ros::Duration RobotHardwareInterface::getPeriod()
{
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    elapsedTime =
            ros::Duration(currentTime.tv_sec - lastTime.tv_sec
            + (currentTime.tv_nsec - lastTime.tv_nsec) / BILLION);
    lastTime = currentTime;

    return elapsedTime;
}

void RobotHardwareInterface::printDebugInfo(std::string name, double* data) {
    ROS_INFO_STREAM(name << " RIGHT_FRONT_WHEEL_JOINT "  << data[0]);
    ROS_INFO_STREAM(name << " RIGHT_MIDDLE_WHEEL_JOINT " << data[1]);
    ROS_INFO_STREAM(name << " RIGHT_BACK_WHEEL_JOINT "   << data[2]);
    ROS_INFO_STREAM(name << " LEFT_FRONT_WHEEL_JOINT "   << data[3]);
    ROS_INFO_STREAM(name << " LEFT_MIDDLE_WHEEL_JOINT "  << data[4]);
    ROS_INFO_STREAM(name << " LEFT_BACK_WHEEL_JOINT "    << data[5]);
}
