#ifndef GNC_CONTROL_DRIVE_HWIN_H
#define GNC_CONTROL_DRIVE_HWIN_H

#include <hardware_interface/joint_state_interface.h>   // for reading the state of the joints (position, velocity, effort)
#include <hardware_interface/joint_command_interface.h> // for sending commands to the joints
#include <controller_manager/controller_manager.h>      // to have a controller in our class
#include <hardware_interface/robot_hw.h>                // needed to interact with ros_control
#include <ros/ros.h>
#include <string>
#include <gnc_control/odrive.hpp>

struct DriveHwinSettings
{
    std::string leftWheelNames[2];
    std::string rightWheelNames[2];
    std::string leftWheelAxis[2]; // axis for odrive, axis 0 = motor 0, axis 1 = motor 1
    std::string rightWheelAxis[2];

    float encoderTicksPerRevolution = 1000.0f;
    int rosLoopRate = 30;
    bool debugMode = false;
};

class DriveHwin : public hardware_interface::RobotHW
{
public:
    DriveHwin(ros::NodeHandle *, const DriveHwinSettings &);
    ~DriveHwin();
    void read();
    void write();
    void modifySettings(const DriveHwinSettings &);

    ros::Time getTime();
    ros::Duration getPeriod();

private:
    // For reading the state of the wheels, and sending desired velocity (commands) to wheels
    hardware_interface::JointStateInterface jointStateInterface;
    hardware_interface::VelocityJointInterface velocityJointInterface;

    void registerStateHandlers();
    void registerJointVelocityHandlers(); // Eventually we will use 6 DOF arm and use position controllers
    void printDebugInfo(std::string name, double *data);

    // Wheel functionality
    int initializeOdrive();
    int rebootOdrive(); 

    DriveHwinSettings driveSettings;
    ros::NodeHandle *nh;

    // odrive, motor controller stuff
    odrive *od;
    int publishOdriveData(odrive_endpoint *, Json::Value, ros::Publisher);
    float calculateVelocity(int ticks);

    ros::Duration elapsedTime;
    struct timespec lastTime;
    struct timespec currentTime;

    // Robot state: indices 0-5 for wheels, indices 6-10 for arm
    double cmd[4];

    double pos[4];
    double vel[4];
    double eff[4];

    static constexpr double BILLION = 1000000000.0; // nanoseconds to seconds
};

#endif
