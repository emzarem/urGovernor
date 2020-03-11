#include <ros/ros.h>

#include "urVision/ObjectTracker.h"

// Srv and msg types
#include <urGovernor/KinematicsTest.h>
#include <urGovernor/MotorConfigTest.h>
#include <vector>

// Parameters to read from configs
std::string fetchWeedServiceName;

static int testIndex = 0;

int motorSpeedDegS;
int motorAccelDegSS;

// DO NOT CHANGE THIS!
Object tests[] = {
    {0,0,55,0},
    {17,15,1,0},
    {-17,15,1,0},
    {-17,-26,1,0},
    {17,-26,1,0},
    {0,0,2,0},
    {0,0,55,0},
};

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;

    if (!nodeHandle.getParam("motor_speed_deg_s", motorSpeedDegS)) return false;
    if (!nodeHandle.getParam("motor_accel_deg_s_s", motorAccelDegSS)) return false;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deltaTest_node");
    ros::NodeHandle nodeHandle("~");

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for deltaTest.");
        ros::requestShutdown();
    }

    ros::ServiceClient configClient = nodeHandle.serviceClient<urGovernor::MotorConfigTest>("/kinematicsTest/mtr_config");
    ros::service::waitForService("/kinematicsTest/mtr_config");
    urGovernor::MotorConfigTest configSrv;

    configSrv.request.speed = motorSpeedDegS;
    configSrv.request.accel = motorAccelDegSS;

    if (!configClient.call(configSrv))
    {
        ROS_ERROR("Could not configure motors for deltaTest.");
        ros::requestShutdown();
    }

    ros::ServiceClient motorClient = nodeHandle.serviceClient<urGovernor::KinematicsTest>("/kinematicsTest/move_to_coords");
    ros::service::waitForService("/kinematicsTest/move_to_coords");
    urGovernor::KinematicsTest motorSrv;

    ros::Rate loopRate(0.5);
    while (ros::ok())
    {
        /* Increment test index */
        if (testIndex >= sizeof(tests)/sizeof(Object))
        {
            break;
        }
        else
        {
            motorSrv.request.x_coord = tests[testIndex].x;
            motorSrv.request.y_coord = tests[testIndex].y;
            motorSrv.request.z_coord = tests[testIndex].z;

            if (!motorClient.call(motorSrv))
            {
                ROS_ERROR("Could not run motors for deltaTest.");
                ros::requestShutdown();
            }
        }

        testIndex++;

        loopRate.sleep();
    }

    ROS_INFO("Delta Test DONE!.");

    return 0;
}
