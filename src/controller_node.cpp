#include <ros/ros.h>

// For kinematics
#include "deltaRobot.h"

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

    /* Initializing Kinematics */
    // Set tool offset (tool id == 0, x, y, z )
    robot_tool_offset(0, 0, 0, 0);
    // Default deltarobot setup
    deltarobot_setup();

    // Main loop
    ros::Rate loopRate(queryRate);
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
            int x_coord = (int32_t)srv.response.weed.x_cm;
            int y_coord = (int32_t)srv.response.weed.y_cm;
            // z = 0 IS AT THE GROUND (z = is always positive)
            int z_coord = soilOffset;
            
            /* Calculate angles for Delta arm */
            robot_position((float)x_coord, (float)x_coord, (float)z_coord); 

            // Get the resulting angles
            struct ArmAngles angles = getArmAngles();
            int angle1Deg = angles.angleB1;
            int angle2Deg = angles.angleB2;
            int angle3Deg = angles.angleB3;

            ROS_INFO("Delta for coords (%i, %i, %i) [cm] -> (%i, %i, %i) [degrees]",
                        x_coord, y_coord, z_coord, angle1Deg, angle2Deg, angle3Deg);

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