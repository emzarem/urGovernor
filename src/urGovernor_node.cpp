#include <ros/ros.h>

// Shared lib
#include "SerialPacket.h"

// For kinematics
#include "deltaRobot.h"

// Srv and msg types
#include <urGovernor/FetchWeed.h>
#include <urGovernor/MarkUprooted.h>
#include <urVision/weedDataArray.h>
#include <urGovernor/SerialWrite.h>
#include <urGovernor/SerialRead.h>

// Parameters to read from configs
std::string fetchWeedServiceName;
std::string markUprootedServiceName;
float overallRate;

std::string serialServiceWriteName;
std::string serialServiceReadName;

int restAngle1, restAngle2, restAngle3;
float cartesianLimitXMax, cartesianLimitXMin, cartesianLimitYMax, cartesianLimitYMin;
float angleLimit;

const int relativeAngleFlag = false;

int soilOffset = 0;

// Time to actuate end-effector
double endEffectorTime = 0;

// Connections to Serial interface services
ros::ServiceClient serialWriteClient;
ros::ServiceClient serialReadClient;

const int logFetchWeedInterval = 5;

int serialTimeoutMs;

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;
    if (!nodeHandle.getParam("mark_uprooted_service", markUprootedServiceName)) return false;
    
    if (!nodeHandle.getParam("controller_overall_rate", overallRate)) return false;

    if (!nodeHandle.getParam("rest_angle_1", restAngle1)) return false;
    if (!nodeHandle.getParam("rest_angle_2", restAngle2)) return false;
    if (!nodeHandle.getParam("rest_angle_3", restAngle3)) return false;

    if (!nodeHandle.getParam("cartesian_limit_x_max", cartesianLimitXMax)) return false;
    if (!nodeHandle.getParam("cartesian_limit_x_min", cartesianLimitXMin)) return false;
    if (!nodeHandle.getParam("cartesian_limit_y_max", cartesianLimitYMax)) return false;
    if (!nodeHandle.getParam("cartesian_limit_y_min", cartesianLimitYMin)) return false;

    if (!nodeHandle.getParam("angle_limit", angleLimit)) return false;

    if (!nodeHandle.getParam("end_effector_time_s", endEffectorTime)) return false;

    if (!nodeHandle.getParam("soil_offset", soilOffset)) return false;

    if (!nodeHandle.getParam("serial_output_service", serialServiceWriteName)) return false;
    if (!nodeHandle.getParam("serial_input_service", serialServiceReadName)) return false;

    if (!nodeHandle.getParam("serial_timeout_ms", serialTimeoutMs)) return false;
    
    return true;
}

bool actuateArmAngles(int angle1Deg, int angle2Deg, int angle3Deg)
{
    urGovernor::SerialWrite serialWrite;
    urGovernor::SerialRead serialRead;   
    SerialUtils::CmdMsg msg = {0};
    std::vector<char> buff;

    msg.is_relative = relativeAngleFlag;
    msg.m1_angle = angle1Deg;
    msg.m2_angle = angle2Deg;
    msg.m3_angle = angle3Deg;
    msg.motors_done = 0;
    
    SerialUtils::pack(buff, msg);

    serialWrite.request.command = std::string(buff.begin(), buff.end());

    // Send angles to HAL (via calling the serial WRITE client)
    if (serialWriteClient.call(serialWrite))
    {
        ros::Rate loopRate( 1.0 / (serialTimeoutMs / 1000.0));
        while (ros::ok())
        {
            // Wait for arm done
            // This is done by calling the serial READ client
            // This should block until we get a CmdMsg FROM the serial line
            if (serialReadClient.call(serialRead))
            {
                std::vector<char> v(serialRead.response.command.begin(), serialRead.response.command.end());
                
                msg.motors_done = 0;
                // Unpack response from read
                SerialUtils::unpack(v, msg);

                // This should indicate that we are done
                if (msg.motors_done)
                {
                    ROS_INFO("Motor callback received.");
                    return 1;
                }
                else 
                {
                    ROS_ERROR("Motors did not return with (motors_done == true)");
                    return 0;
                }
            }
            else
            {
                ROS_DEBUG("Timed out waiting for response from Teensy ... retrying ...");
            }
            loopRate.sleep();
        }
    }
    else
    {
        ROS_ERROR("Serial write to set motors was NOT successful.");
    }
    
    return 0;
}

bool updateArmAngles(int angle1Deg, int angle2Deg, int angle3Deg)
{
    urGovernor::SerialWrite serialWrite;
    urGovernor::SerialRead serialRead;   
    SerialUtils::CmdMsg msg = {0};
    std::vector<char> buff;

    msg.is_relative = relativeAngleFlag;
    msg.m1_angle = angle1Deg;
    msg.m2_angle = angle2Deg;
    msg.m3_angle = angle3Deg;
    msg.motors_done = 0;
    
    SerialUtils::pack(buff, msg);

    serialWrite.request.command = std::string(buff.begin(), buff.end());

    // Send angles to HAL (via calling the serial WRITE client)
    if (serialWriteClient.call(serialWrite))
    {
        return true;
    }
    else
    {
        ROS_ERROR("Serial write to update motor angles was NOT successful.");
    }
    
    return false;
}

/* This function does the physical actuation for 'uprooting' a weed
 *      This controls the strategy for going for a specific weed.
 * 
 */
bool doUprootWeed(float targetX, float targetY, float targetZ, float targetSize)
{
    // Time this whole operation
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();

    // IF cartesian coordinate are out of range
    if (targetX > cartesianLimitXMax ||
        targetX < cartesianLimitXMin ||
        targetY > cartesianLimitYMax ||
        targetY < cartesianLimitYMin )
    {
        ROS_INFO("COORDS OUT OF RANGE of delta arm [(x,y,size)=(%.1f,%.1f,%.1f)]",targetX,targetY,targetSize);
        return false;
    }

    /* Create coordinates in the Delta Arm Reference
    *   This conversion requires a 'rotation matrix' 
    *   to be applied to comply with Delta library coordinates.
    *   x' = x*cos(theta) - y*sin(theta)
    *   y' = x*sin(theta) + y*cos(theta)
    * Based on our setup, theta = +60 degrees AND X and Y coordinates are switched
    */
    float x_coord = (float)(targetY*(0.5) - (targetX)*(0.866));
    float y_coord = (float)(targetY*(0.866) + (targetX)*(0.5));
    float z_coord = (float)targetZ + soilOffset;    // z = 0 IS AT THE GROUND (z = is always positive)

    /* Calculate angles for Delta arm */
    robot_position(x_coord, y_coord, z_coord); 

    int angle1Deg,angle2Deg,angle3Deg;

    // Get the resulting angles from kinematics
    if (getArmAngles(&angle1Deg, &angle2Deg, &angle3Deg))
    {
        // IF calculated angles are out of range
        if (angle1Deg > angleLimit ||
            angle2Deg > angleLimit ||
            angle3Deg > angleLimit)
        {
            ROS_ERROR("ANGLES OUT OF RANGE of delta arm [(a1,a2,a3)=(%i,%i,%i)]",angle1Deg,angle2Deg,angle3Deg);
            return false;
        }

        ROS_INFO("Delta for coords (%.1f,%.1f,%.1f) [cm] -> (%i,%i,%i) [degrees]",
                    targetX, targetY, targetZ, 
                    angle1Deg, angle2Deg, angle3Deg);

        // First, come up to 0. This ensures we don't hit anything on the way
        if (!actuateArmAngles(0, 0, 0))
        {
            ROS_ERROR("Could not bring arms to zero.");
            ros::requestShutdown();
            return false;
        }

        // GO to our target
        if (actuateArmAngles(angle1Deg, angle2Deg, angle3Deg))
        {
            // Wait for end-effectors
            ros::Duration(endEffectorTime).sleep();

            // Back up to 0
            if (!actuateArmAngles(0, 0, 0))
            {
                ROS_ERROR("Could not bring arms to zero.");
                ros::requestShutdown();
                return false;
            }

            // Back to resting position
            if (!actuateArmAngles(restAngle1, restAngle2, restAngle3))
            {
                ROS_ERROR("Could not Reset arm positions.");
                ros::requestShutdown();
                return false;
            }

            // That's it -- the weed is 'uprooted'
            end_ = ros::WallTime::now();
            double execution_time = (end_ - start_).toNSec() * 1e-6;
            ROS_DEBUG_STREAM("Actuation time for weed (ms): " << execution_time);

            return true;
        }
        else
        {
            ROS_ERROR("Could not actuate motors to specified arm angles");
            ros::requestShutdown();
        }
    }
    else
    {
        ROS_ERROR("Could not get arm angles.");
    }

    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "urGovernor_node");
    ros::NodeHandle nh;
    ros::NodeHandle nodeHandle("~");

    int fetchWeedLogs = 0;

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for urGovernor_node.");
        ros::requestShutdown();
    }

    serialWriteClient = nh.serviceClient<urGovernor::SerialWrite>(serialServiceWriteName);
    ros::service::waitForService(serialServiceWriteName);

    serialReadClient = nh.serviceClient<urGovernor::SerialRead>(serialServiceReadName);
    ros::service::waitForService(serialServiceReadName);

    // Subscribe to service from tracker
    ros::ServiceClient fetchWeedClient = nh.serviceClient<urGovernor::FetchWeed>(fetchWeedServiceName);
    ros::service::waitForService(fetchWeedServiceName);
    urGovernor::FetchWeed fetchWeedSrv;

    // Subscribe to second service from tracker
    ros::ServiceClient markUprootedClient = nh.serviceClient<urGovernor::MarkUprooted>(markUprootedServiceName);
    ros::service::waitForService(markUprootedServiceName);
    urGovernor::MarkUprooted markUprootedSrv;


    /* Initializing Kinematics */
    // Set tool offset (tool id == 0, x, y, z )
    robot_tool_offset(0, 0, 0, 0);
    // Default deltarobot setup
    deltarobot_setup();

    // SET arms to initial position
    if (!actuateArmAngles(restAngle1, restAngle2, restAngle3))
    {
        ROS_ERROR("Could not Initialize arm positions.");
        ros::requestShutdown();
    }

    // Sleep for a bit at start up to let vision pipeline get going
    ros::Duration(5.0).sleep();

    /* 
     * Main loop for urGovernor
     */
    ros::Rate loopRate(overallRate);
    while (ros::ok())
    {
        fetchWeedSrv.request.caller = 1;
        // Call for a new weed
        if (fetchWeedClient.call(fetchWeedSrv))
        {
            // UpRoot this weed!
            // This call will block until the arm is back in it's rest position
            markUprootedSrv.request.success = doUprootWeed(fetchWeedSrv.response.weed.x_cm, fetchWeedSrv.response.weed.y_cm, fetchWeedSrv.response.weed.z_cm, fetchWeedSrv.response.weed.size_cm);

            // Mark this weed as uprooted
            markUprootedSrv.request.tracking_id = fetchWeedSrv.response.tracking_id;

            if (!markUprootedClient.call(markUprootedSrv))
            {
                ROS_ERROR("Governor -- Could not mark weed as uprooted (call to Tracker).");
            }
        }
        else
        {
            if (fetchWeedLogs % logFetchWeedInterval == 1)
            {
                ROS_INFO("Governor -- no weeds are current.");
            }
            fetchWeedLogs++;
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
