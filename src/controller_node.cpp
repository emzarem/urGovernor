#include <ros/ros.h>

#include "DeltaInverseKinematics.h"

// Srv and msg types
#include <urGovernor/FetchWeed.h>
#include <urVision/weedDataArray.h>

std::string fetchWeedServiceName;
float queryRate;

// Kinematics Parameters
double angleB1;
double angleB2;
double angleB3;

// All parameters in mm !
double armLength;
double rodLength;
double bassTri;
double platformTri;

const double heightOffGround = 50;

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;
    if (!nodeHandle.getParam("controller_query_rate", queryRate)) return false;

    if (!nodeHandle.getParam("arm_length", armLength)) return false;
    if (!nodeHandle.getParam("rod_length", rodLength)) return false;
    if (!nodeHandle.getParam("bass_tri", bassTri)) return false;
    if (!nodeHandle.getParam("platform_tri", platformTri)) return false;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle nodeHandle("~");

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for controller_node.");
        ros::requestShutdown();
    }

    // Subscribe to service from governor
    ros::ServiceClient client = nh.serviceClient<urGovernor::FetchWeed>(fetchWeedServiceName);
    ros::service::waitForService(fetchWeedServiceName);
    urGovernor::FetchWeed srv;

    // Setup Kinematics
    DeltaInverseKinematics deltaKinematics(&angleB1, &angleB2, &angleB3, armLength, rodLength, bassTri, platformTri);

    // TODO: Set Offsets and limits
    // void setOffsets(double X, double Y, double Z);
    // This fuctions is used to set the Offsets of X, Y and Z.
    // void setLimits(double upperX, double upperY, double upperZ, double lowerX, double lowerY, double lowerZ);
    // This fuctions is used to set the angle limits of each motor.

    // Main loop
    ros::Rate loopRate(queryRate); // 10 hz
    while (ros::ok())
    {
        srv.request.caller = 1;
        // Call for a new weed
        if (client.call(srv))
        {
            ROS_INFO("CONTROLLER -- got weed at (%i, %i)", (int32_t)srv.response.weed.x_cm, (int32_t)srv.response.weed.y_cm);

            deltaKinematics.set((int32_t)srv.response.weed.x_cm*10, (int32_t)srv.response.weed.y_cm*10, heightOffGround);  
            ROS_INFO("Set Delta Angles to : (%i, %i, %i)", int(angleB1 * 180 / 3.14), int(angleB2 * 180 / 3.14), int(angleB3 * 180 / 3.14));

            // Send angles to HAL
            // Serial.println(String(angleB1)+","+String(angleB2* 180 / 3.14)+","+String(angleB3* 180 / 3.14));
        }
        else
        {
            ROS_DEBUG("No reponse from FetchWeed service");
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}