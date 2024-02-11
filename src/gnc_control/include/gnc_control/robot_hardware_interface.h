#ifndef GNC_CONTROL_ROBOT_HARDWARE_INTERFACE_H
#define GNC_CONTROL_ROBOT_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>  // for reading the state of the joints (position, velocity, effort)
#include <hardware_interface/joint_command_interface.h> // for sending commands to the joints
#include <controller_manager/controller_manager.h> // to have a controller in our class
#include <hardware_interface/robot_hw.h> // needed to interact with ros_control
#include <ros/ros.h>
#include <string>
#include <gnc_control/roboclaw.h>

struct RobotHwinSettings
{
    std::string leftWheelNames[3];
    std::string rightWheelNames[3];
    std::string armJointNames[6];

    int leftWheelRoboclawAddresses[3]; // Corresponding wheel to roboclaw address
    int rightWheelRoboclawAddresses[3];
    int armRoboclawAddresses[6];

    uint8_t maxEffortValue = 126;
    int rosLoopRate = 10;
    int maxRetries = 3;
    bool debugMode = false;
};

class RobotHardwareInterface : public hardware_interface::RobotHW
{
public:
    RobotHardwareInterface(ros::NodeHandle*, RobotHwinSettings*);
    ~RobotHardwareInterface();
    void read(Roboclaw*);
    void write(Roboclaw*);

    ros::Time getTime();
    ros::Duration getPeriod();

private:
    RobotHwinSettings* robotSettings;

    // For reading the state of the wheels, and sending desired velocity (commands) to wheels
    hardware_interface::JointStateInterface jointStateInterface;
    hardware_interface::VelocityJointInterface velocityJointInterface;

    void registerStateHandlers();
    void registerJointVelocityHandlers(); // Eventually we will use 6 DOF arm and use position controllers
    void printDebugInfo(std::string name, double* data);

    // Wheel functionality
    void sendCommandToWheels(Roboclaw*);
    void scaleWheelCommands();
    void scaleArmCommands(uint8_t*);
    void getVelocityFromWheels();
    double convertPulsesToRadians(double vel);

    // Arm functionality
   void sendCommandToArm(Roboclaw*);

    ros::Duration elapsedTime;
    struct timespec lastTime;
    struct timespec currentTime;
    int zeroCmdVelCount;

    // Robot state: indices 0-5 for wheels, indices 6-10 for arm
    double cmd[12];
    uint8_t cmdToSend[12]; // passed to roboclaw

    double pos[12];
    double vel[12];
    double eff[12];

    static constexpr double BILLION = 1000000000.0; // nanoseconds to seconds
};

#endif
