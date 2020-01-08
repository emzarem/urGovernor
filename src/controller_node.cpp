#include <ros/ros.h>

#include "DeltaInverseKinematics.h"

// Srv and msg types
#include <urGovernor/FetchWeed.h>
#include <urVision/weedDataArray.h>

const float COS_30_DEG = 0.866025;

std::string fetchWeedServiceName;
float queryRate;

/* START Kinematics Parameters */
int relativeAngleFlag = false;

// Bicep angles (in radians)
double angleB1;
double angleB2;
double angleB3;

// Delta Arm dimensions
// All parameters in mm!
double armLength = 0;
double rodLength = 0;
double platformRadius = 0;
double endEffectorRadius = 0;

double baseToFloor = 0;
int soilOffset = 0;

/* END Kinematics Parameters */

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;
    if (!nodeHandle.getParam("controller_query_rate", queryRate)) return false;

    if (!nodeHandle.getParam("bicep_length", armLength)) return false;
    if (!nodeHandle.getParam("forearm_length", rodLength)) return false;
    if (!nodeHandle.getParam("base_platform_radius", platformRadius)) return false;
    if (!nodeHandle.getParam("end_effector_platform_radius", endEffectorRadius)) return false;

    if (!nodeHandle.getParam("base_to_floor", baseToFloor)) return false;
    if (!nodeHandle.getParam("soil_offset", soilOffset)) return false;
    
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

    // Convert radii to equivalent equilateral triangle side lengths
    int bassTri = 2*platformRadius*COS_30_DEG;
    int platformTri = 2*endEffectorRadius*COS_30_DEG;

    // Setup (Inverse) Kinematics instance
    DeltaInverseKinematics deltaKinematics(&angleB1, &angleB2, &angleB3, armLength, rodLength, bassTri, platformTri);

    // TODO: Used to set angle limits for the motor
    // void setLimits(double upperB1, double upperB2, double upperB3, double lowerB1, double lowerB2, double lowerB3)

    // TODO: Offsets not really working, just set to 0 right now
    deltaKinematics.setOffsets(0, 0, 0);

    // Main loop
    ros::Rate loopRate(queryRate); // 10 hz
    while (ros::ok())
    {
        srv.request.caller = 1;
        // Call for a new weed
        if (client.call(srv))
        {
            ROS_INFO("CONTROLLER -- got weed at (%i, %i)", (int32_t)srv.response.weed.x_cm, (int32_t)srv.response.weed.y_cm);

            /* Create coordinates in the Delta Arm Reference  */
            // All coordinates in mm
            // TODO: need to do some conversion here (arm is centered on (0,0) in the middle)
            int x_coord = (int32_t)srv.response.weed.x_cm*10;
            int y_coord = (int32_t)srv.response.weed.y_cm*10;
            // z = 0 is at the base (z = is always negative)
            int z_coord = soilOffset - baseToFloor;
            
            /* Calculate angles for Delta arm */
            deltaKinematics.set(30, 30, z_coord);

            int angle1Deg = int(angleB1 * 180 / 3.14);
            int angle2Deg = int(angleB2 * 180 / 3.14);
            int angle3Deg = int(angleB3 * 180 / 3.14);

            ROS_INFO("Set Delta Angles to : (%i, %i, %i)", angle1Deg, angle2Deg, angle3Deg);

            // Send angles to HAL
            // printf("%i,%i,%i,%i\n", angle1Deg, angle2Deg, angle3Deg, relativeAngleFlag);
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