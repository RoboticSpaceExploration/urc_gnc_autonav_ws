#include "gnc_control/drive_hwin.h"

DriveHwin::DriveHwin(ros::NodeHandle *nh, const DriveHwinSettings &settings)
    : driveSettings(settings), nh(nh)
{
    registerStateHandlers();
    registerJointVelocityHandlers();

    if (initializeOdrive())
    {
        ROS_ERROR("Couldn't initialize odrive. Exiting program");
        ros::shutdown();
    }

    // initialize time
    clock_gettime(CLOCK_MONOTONIC, &lastTime);

    for (int i = 0; i < 4; i++)
        cmd[i] = pos[i] = vel[i] = eff[i] = 0;
}

DriveHwin::~DriveHwin()
{
    // clean up odrive
    for (int i = 0; i < od->target_sn.size(); i++)
    {
        od->endpoint.at(i)->remove();
        delete od->endpoint.at(i);
    }
}

void DriveHwin::modifySettings(const DriveHwinSettings &settings)
{
    driveSettings = settings;
}

void DriveHwin::read()
{
    // Update all odrive targets
    for (int i = 0; i < od->target_sn.size(); i++)
    {
        odrive_endpoint *endpoint = NULL;
        Json::Value odrive_json;

        endpoint = od->endpoint.at(i);
        odrive_json = od->json.at(i);

        // update watchdog
        execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");
        execOdriveFunc(endpoint, odrive_json, "axis1.watchdog_feed");
        // Publish status message
        this->publishOdriveData(endpoint, odrive_json, od->odrive_pub.at(i));
    }
}

void DriveHwin::write()
{
    // Update all odrive targets
    for (int i = 0; i < od->target_sn.size(); i++)
    {
        // should only be 2 motor controllers maximum, left then right
        if (i >= 2)
            break;


        odrive_endpoint *endpoint = NULL;
        Json::Value odrive_json;

        endpoint = od->endpoint.at(i);
        odrive_json = od->json.at(i);

        uint32_t odrive_error0;
        uint32_t odrive_error1;
        readOdriveData(endpoint, odrive_json, string("axis0.error"), odrive_error0);
        readOdriveData(endpoint, odrive_json, string("axis1.error"), odrive_error1);
        if (odrive_error0 != 0)
        {
            execOdriveFunc(endpoint, odrive_json, "axis0.clear_errors");
        }
        if (odrive_error1 != 0)
        {
            execOdriveFunc(endpoint, odrive_json, "axis1.clear_errors");
        }

        // update watchdog
        execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");
        execOdriveFunc(endpoint, odrive_json, "axis1.watchdog_feed");

        // variables for sending to motor controller
        // ros control multiplies cmd_vel by 5, 31.4 is     
        float axis0_fval = (float)((cmd[2 * i] * 31.4) / 5); 
        float axis1_fval = (float)((cmd[1 + (2 * i)] * 31.4) / 5.0);
        ROS_INFO_STREAM("AXIS 0 CMD: " << axis0_fval);

        // build command to send
        std::string cmdAxis0 = "axis0.controller.input_vel";
        std::string cmdAxis1 = "axis1.controller.input_vel";

        // send drive cmd to motor controller
        writeOdriveData(endpoint, odrive_json, cmdAxis0, axis0_fval);
        writeOdriveData(endpoint, odrive_json, cmdAxis1, axis1_fval);
    }

    // cmd[0] = cmd[1] = cmd[2] = cmd[3] = 0;
}

void DriveHwin::registerStateHandlers()
{
    ROS_INFO("Registering JointStateHandlers");
    // wheel state handlers
    hardware_interface::JointStateHandle wheelLeftFront(
        driveSettings.leftWheelNames[0], &pos[0], &vel[0], &eff[0]);
    jointStateInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointStateHandle wheelLeftBack(
        driveSettings.leftWheelNames[1], &pos[1], &vel[1], &eff[1]);
    jointStateInterface.registerHandle(wheelLeftBack);

    hardware_interface::JointStateHandle wheelRightFront(
        driveSettings.rightWheelNames[0], &pos[2], &vel[2], &eff[2]);
    jointStateInterface.registerHandle(wheelRightFront);

    hardware_interface::JointStateHandle wheelRightBack(
        driveSettings.rightWheelNames[1], &pos[3], &vel[3], &eff[3]);
    jointStateInterface.registerHandle(wheelRightBack);

    registerInterface(&jointStateInterface);
}

void DriveHwin::registerJointVelocityHandlers()
{
    ROS_INFO("Registering JointVelocityHandlers");
    hardware_interface::JointHandle wheelLeftFront(
        jointStateInterface.getHandle(driveSettings.leftWheelNames[0]), &cmd[0]);
    velocityJointInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointHandle wheelLeftBack(
        jointStateInterface.getHandle(driveSettings.leftWheelNames[1]), &cmd[1]);
    velocityJointInterface.registerHandle(wheelLeftBack);

    hardware_interface::JointHandle wheelRightFront(
        jointStateInterface.getHandle(driveSettings.rightWheelNames[0]), &cmd[2]);
    velocityJointInterface.registerHandle(wheelRightFront);

    hardware_interface::JointHandle wheelRightBack(
        jointStateInterface.getHandle(driveSettings.rightWheelNames[1]), &cmd[3]);
    velocityJointInterface.registerHandle(wheelRightBack);

    registerInterface(&velocityJointInterface);
}

int DriveHwin::initializeOdrive()
{
    std::string od_sn;
    std::string od_cfg;

    ROS_INFO("Starting ODrive...");
    od = new odrive();
    nh->param<std::string>("od_sn", od_sn, "0x00000000");
    nh->param<std::string>("od_cfg", od_cfg, "");

    ROS_INFO_STREAM("od_sn: " << od_sn);
    ROS_INFO_STREAM("od_cfg: " << od_cfg);

    // Get device serial number string
    if (!nh->getParam("od_sn", od_sn))
    {
        ROS_ERROR("Failed to get serial number from rosparam: %s!", od_sn.c_str());
        return -1;
    }

    // parse sn string for comma seperated values / more than one targets
    std::stringstream ssn(od_sn);
    while (ssn.good())
    {
        string substr;
        getline(ssn, substr, ',');
        od->target_sn.push_back(substr);
    }

    // parse cfg string for comma seperated values / more than one targets
    std::stringstream scfg(od_cfg);
    while (scfg.good())
    {
        string substr;
        getline(scfg, substr, ',');
        od->target_cfg.push_back(substr);
    }

    if (od->target_sn.size() != od->target_cfg.size())
    {
        ROS_ERROR_STREAM("* Amount of configuration files do not match amount of Serial Numbers! sn.size() = "
                         << od->target_sn.size() << " cfg.size() = " << od->target_cfg.size());
        return -1;
    }

    // Initialize publisher/subscriber for each target
    ROS_INFO("%d odrive instances:", (int)od->target_sn.size());
    for (int i = 0; i < od->target_sn.size(); i++)
    {
        ROS_INFO("- Instance %d: SN %s - cfg %s",
                 i, od->target_sn.at(i).c_str(), od->target_cfg.at(i).c_str());
        od->odrive_pub.push_back(
            nh->advertise<gnc_control::odrive_msg>("odrive_msg_" + od->target_sn.at(i), 100));
        // od->odrive_sub.push_back(
        //     nh->subscribe("odrive_ctrl_" + od->target_sn.at(i), 10, msgCallback));

        // Get odrive endpoint instance
        od->endpoint.push_back(new odrive_endpoint());

        // Enumarate Odrive target
        if (od->endpoint.at(i)->init(stoull(od->target_sn.at(i), 0, 16)))
        {
            ROS_ERROR("* Device not found!");
            return 1;
        }

        // Read JSON from target
        Json::Value odrive_json;
        if (getJson(od->endpoint.at(i), &odrive_json))
        {
            return -1;
        }

        od->json.push_back(odrive_json);

        // Process configuration file
        execOdriveFunc(od->endpoint.at(i), od->json.at(i), 
            "axis0.clear_errors");
        execOdriveFunc(od->endpoint.at(i), od->json.at(i), 
            "axis1.clear_errors");
        updateTargetConfig(od->endpoint.at(i), od->json.at(i), od->target_cfg.at(i));

        // enter closed loop drive after configuration
        ROS_INFO("Entering closed loop control with motors");
        int closed_loop_control = AXIS_STATE_CLOSED_LOOP_CONTROL;
        writeOdriveData(od->endpoint.at(i), od->json.at(i),
                        "axis0.requested_state", closed_loop_control);
        writeOdriveData(od->endpoint.at(i), od->json.at(i),
                        "axis1.requested_state", closed_loop_control);
    }

    return 0;
}

float DriveHwin::calculateVelocity(int ticks)
{
    return 0.0f;
}

/**
 *
 * Publise odrive message to ROS
 * @param endpoint odrive enumarated endpoint
 * @param odrive_json target json
 * @param odrive_pub ROS publisher
 * return ODRIVE_OK in success
 *
 */
int DriveHwin::publishOdriveData(odrive_endpoint *endpoint, Json::Value odrive_json, ros::Publisher odrive_pub)
{
    uint16_t u16val;
    uint32_t u32val;
    float fval;
    gnc_control::odrive_msg msg;

    // Collect data
    readOdriveData(endpoint, odrive_json, string("axis0.error"), u32val);
    msg.error0 = u32val;
    readOdriveData(endpoint, odrive_json, string("axis1.error"), u32val);
    msg.error1 = u32val;
    readOdriveData(endpoint, odrive_json, string("axis0.current_state"), u32val);
    msg.state0 = u32val;
    readOdriveData(endpoint, odrive_json, string("axis1.current_state"), u32val);
    msg.state1 = u32val;

    // Publish message
    odrive_pub.publish(msg);

    return ODRIVE_OK;
}

ros::Time DriveHwin::getTime()
{
    return ros::Time::now();
}

ros::Duration DriveHwin::getPeriod()
{
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    elapsedTime =
        ros::Duration(currentTime.tv_sec - lastTime.tv_sec + (currentTime.tv_nsec - lastTime.tv_nsec) / BILLION);
    lastTime = currentTime;

    return elapsedTime;
}