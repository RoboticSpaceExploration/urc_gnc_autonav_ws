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

#include <gnc_control/arm_hardware_interface.h>
#include <gnc_control/roboclaw.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_hardware_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO_STREAM("Loading arm_control_node");
    ArmHardwareInterface arm(&nh);
    controller_manager::ControllerManager cm(&arm);


    RoboclawSettings rbSettings;
    rbSettings.serialPortAddress = "/dev/ttyTHS2";
    Roboclaw roboclaw(&rbSettings);

    ros::Rate rate(10);

    while (ros::ok()) {
        arm.readFromArm(&roboclaw);
        cm.update(arm.get_time(), arm.get_period());
        arm.writeToArm(&roboclaw);
        rate.sleep();
    }

    ROS_INFO("Shutting down arm hardware interface");

    return 0;
}

ArmHardwareInterface::ArmHardwareInterface(ros::NodeHandle* nh)
{
    ROS_INFO("Registering ros_control handlers");
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    for (int i = 0; i < 6; i++)
        cmd[i] = vel[i] = pos[i] = eff[i] = 0;
}

ArmHardwareInterface::~ArmHardwareInterface() { }

void ArmHardwareInterface::registerStateHandlers()
{
    armJoints[0] = "base_to_shoulder_joint";
    armJoints[1] = "shoulder_to_arm_joint";
    armJoints[2] = "arm_to_yzwrist_joint";
    armJoints[3] = "yzwrist_to_xywrist_joint";
    armJoints[4] = "xywrist_to_xzwrist_joint";
    armJoints[5] = "xzwrist_to_grip_joint";
    hardware_interface::JointStateHandle stateHandleA(
        armJoints[0], &pos[0], &vel[0], &eff[0]);
    jntStateInterface.registerHandle(stateHandleA);

    hardware_interface::JointStateHandle stateHandleB(
        armJoints[1], &pos[1], &vel[1], &eff[1]);
    jntStateInterface.registerHandle(stateHandleB);

    hardware_interface::JointStateHandle stateHandleC(
        armJoints[2], &pos[2], &vel[2], &eff[2]);
    jntStateInterface.registerHandle(stateHandleC);

    hardware_interface::JointStateHandle stateHandleD(
        armJoints[3], &pos[3], &vel[3], &eff[3]);
    jntStateInterface.registerHandle(stateHandleD);

    hardware_interface::JointStateHandle stateHandleE(
        armJoints[4], &pos[4], &vel[4], &eff[4]);
    jntStateInterface.registerHandle(stateHandleE);

    hardware_interface::JointStateHandle stateHandleF(
        armJoints[5], &pos[5], &vel[5], &eff[5]);
    jntStateInterface.registerHandle(stateHandleF);

    registerInterface(&jntStateInterface);
}
void ArmHardwareInterface::registerJointVelocityHandlers()
{
        hardware_interface::JointHandle vel_handle_a(
        jntStateInterface.getHandle(armJoints[0]), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(
        jntStateInterface.getHandle(armJoints[1]), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_b);

    hardware_interface::JointHandle vel_handle_c(
        jntStateInterface.getHandle(armJoints[2]), &cmd[2]);
    jnt_vel_interface.registerHandle(vel_handle_c);

    hardware_interface::JointHandle vel_handle_d(
        jntStateInterface.getHandle(armJoints[3]), &cmd[3]);
    jnt_vel_interface.registerHandle(vel_handle_d);

    hardware_interface::JointHandle vel_handle_e(
        jntStateInterface.getHandle(armJoints[4]), &cmd[4]);
    jnt_vel_interface.registerHandle(vel_handle_e);

    hardware_interface::JointHandle vel_handle_f(
        jntStateInterface.getHandle(armJoints[5]), &cmd[5]);
    jnt_vel_interface.registerHandle(vel_handle_f);

    registerInterface(&jnt_vel_interface);
}

void ArmHardwareInterface::writeToArm(Roboclaw* roboclaw)
{
    uint8_t cmd_send[6];
    ScaleCommands();

    for (int i = 0; i < 6; i++)
    {
        cmd_send[i] = fabs(cmd[i]);
    }

    
    // Linear actuators
    if (cmd[0] > 0)
    {
	    ROS_INFO_STREAM("got here address 131");
        roboclaw->ForwardM1(0x83, 126);
        roboclaw->ForwardM2(0x83, 126);
    }
    else if (cmd[0] < 0)
    {
        roboclaw->BackwardM1(0x83, 126);
        roboclaw->BackwardM2(0x83, 126);
    }
    else
    {
        roboclaw->ForwardM1(0x83, 0);
        roboclaw->ForwardM2(0x83, 0);
    }

    // other motors
    if (cmd[1] >= 0)  // arm base
    {
        roboclaw->ForwardM1(0x84, 60);
        roboclaw->ForwardM2(0x84, 60);
    }
    else
    {
        roboclaw->BackwardM1(0x84, 60);
        roboclaw->BackwardM2(0x84, 60);
    }

    if (cmd[2] >= 0)  // right_middle
        roboclaw->ForwardM1(0x86, cmd_send[2]);
    else
        roboclaw->BackwardM1(0x86,cmd_send[2]);

    if (cmd[3] >= 0)  
        roboclaw->ForwardM1(0x85, cmd_send[3]);
    else
        roboclaw->BackwardM1(0x85, cmd_send[3]);

    if (cmd[4] >= 0)  // right_back
        roboclaw->ForwardM2(0x85, cmd_send[4]);
    else
        roboclaw->BackwardM2(0x85, cmd_send[4]);


    // if (cmd[5] >= 0)  // left_middle not used rn
    //     roboclaw->ForwardM2(0x87, cmd_send[5]);
    // else
    //     roboclaw->BackwardM2(0x87, cmd_send[5]);
    
    for (int i = 0; i < 6; i++)
    {
        cmd[i] = 0;
    }
}

void ArmHardwareInterface::readFromArm(Roboclaw *roboclaw)
{
    // TODO: No support for reading encoders yet
}

void ArmHardwareInterface::ScaleCommands()
{
    int maxVal = 126;
    for (int i = 0; i < 6; i++)
    {
        cmd[i] *= maxVal;
        if (cmd[i] >= maxVal)
            cmd[i] = maxVal;
        else if (cmd[i] <= -maxVal)
            cmd[i] = -maxVal;
    }
}

ros::Time ArmHardwareInterface::get_time()
{
    return ros::Time::now();
}

ros::Duration ArmHardwareInterface::get_period()
{
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time =
            ros::Duration(current_time.tv_sec - last_time.tv_sec
            + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    return elapsed_time;
}

void ArmHardwareInterface::printDebugInfo(std::string name, double* data) {
    // TODO
}
