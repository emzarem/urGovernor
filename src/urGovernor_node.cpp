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

bool doConstantTracking;
float initSleepTime;

const int relativeAngleFlag = false;

int soilOffset = 0;

// Time to actuate end-effector
double endEffectorTime = 0;

// Connections to Serial interface services
ros::ServiceClient serialWriteClient;
ros::ServiceClient serialReadClient;
ros::ServiceClient fetchWeedClient;
ros::ServiceClient markUprootedClient;

const int logFetchWeedInterval = 5;

int serialTimeoutMs;

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;
    if (!nodeHandle.getParam("mark_uprooted_service", markUprootedServiceName)) return false;
    
    if (!nodeHandle.getParam("controller_overall_rate", overallRate)) return false;
    if (!nodeHandle.getParam("do_constant_tracking", doConstantTracking)) return false;
    if (!nodeHandle.getParam("init_sleep_time", initSleepTime)) return false;

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

// Single set point, blocks until it has been reached
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

// Single set point, updates only, returns immediately
bool sendArmAngles(int angle1Deg, int angle2Deg, int angle3Deg)
{
    urGovernor::SerialWrite serialWrite;
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
        ROS_ERROR("Serial write to set motors was NOT successful.");
        return false;
    }
}

// Check for callback from motors
bool checkMotorsDone()
{
    urGovernor::SerialRead serialRead;   
    SerialUtils::CmdMsg msg = {0};

    // Try to read on serial
    if (serialReadClient.call(serialRead))
    {
        std::vector<char> v(serialRead.response.command.begin(), serialRead.response.command.end());
        
        msg.motors_done = 0;
        // Unpack response from read
        SerialUtils::unpack(v, msg);

        // Check if we are done
        if (msg.motors_done)
        {
            return true;
        }
    }

    return false;
}

/* This function does the physical actuation for 'uprooting' a weed
 *      This controls the strategy for going for a specific weed.
 * 
 */
bool doConstantTrackingUproot(urGovernor::FetchWeed& fetchWeedSrv)
{
    // Continually update these angles
    int oldAngle1 = 0,oldAngle2 = 0,oldAngle3 = 0;
    // Store the first tracking_id, if we stray from this, just return from this function
    int tracking_id = fetchWeedSrv.response.tracking_id;

    // Time this whole operation
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();

    // TODO: Do we need this?
    // First, come up to 0
    if (!actuateArmAngles(0, 0, 0))
    {
        ROS_ERROR("Could not bring arms to zero.");
        ros::requestShutdown();
        return false;
    }

    // Do a continual update on the weeds location
    ros::Rate loopRate(overallRate);
    while (ros::ok())
    {
        // IF the arm has reached it's position
        if (checkMotorsDone())
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
        // Otherwise, call fetchWeed to update the coordinates!
        else
        {
            // Make another call to fetchWeed to update the coordinates
            if (!fetchWeedClient.call(fetchWeedSrv))
            {
                // If we lost the weed, return false
                return false;
            }
        
            // IF response has a different tracking_id, return false
            if(fetchWeedSrv.response.tracking_id != tracking_id)
            {
                return false;
            }
        }   

        //// Process the current coordinates
        float targetX = fetchWeedSrv.response.weed.x_cm;
        float targetY = fetchWeedSrv.response.weed.y_cm;
        float targetZ = fetchWeedSrv.response.weed.z_cm;
        float targetSize = fetchWeedSrv.response.weed.size_cm;

        // IF cartesian coordinate are out of range
        if (targetX > cartesianLimitXMax ||
            targetX < cartesianLimitXMin ||
            targetY > cartesianLimitYMax ||
            targetY < cartesianLimitYMin ) {
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
        
        // Get the resulting angles from kinematics
        int angle1Deg, angle2Deg, angle3Deg;
        getArmAngles(&angle1Deg, &angle2Deg, &angle3Deg);

        // IF calculated angles are out of range
        if (angle1Deg > angleLimit ||
            angle2Deg > angleLimit ||
            angle3Deg > angleLimit)
        {
            ROS_ERROR("ANGLES OUT OF RANGE of delta arm [(a1,a2,a3)=(%i,%i,%i)]",angle1Deg,angle2Deg,angle3Deg);
            return false;
        }

        // If any of the angles have changed
        if (angle1Deg != oldAngle1 ||
            angle2Deg != oldAngle2 ||
            angle3Deg != oldAngle3)
        {     
            oldAngle1 = angle1Deg;
            oldAngle2 = angle2Deg;
            oldAngle3 = angle3Deg;

            ROS_INFO("UPDATE coords (%.1f,%.1f,%.1f) [cm] -> (%i,%i,%i) [degrees]",
                targetX, targetY, targetZ, 
                angle1Deg, angle2Deg, angle3Deg);

            // Update the arm angles
            if (!sendArmAngles(angle1Deg, angle2Deg, angle3Deg))
            {
                ROS_ERROR("Could not actuate motors to specified arm angles");
                ros::requestShutdown();
                return false;
            }
        }

        // After send the arm angle update, sleep for the loop rate
        loopRate.sleep();
    }

    return false;
}

/* This function does the physical actuation for 'uprooting' a weed
 *      This controls the strategy for going for a specific weed.
 * 
 */
bool doStaticUproot(urGovernor::FetchWeed& fetchWeedSrv)
{
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
    fetchWeedClient = nh.serviceClient<urGovernor::FetchWeed>(fetchWeedServiceName);
    ros::service::waitForService(fetchWeedServiceName);
    urGovernor::FetchWeed fetchWeedSrv;

    // Subscribe to second service from tracker
    markUprootedClient = nh.serviceClient<urGovernor::MarkUprooted>(markUprootedServiceName);
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

    // Sleep for startup to ensure we get our camera stream
    ros::Duration(initSleepTime).sleep();

    /* 
     * Main loop for urGovernor
     */
    ros::Rate loopRate(overallRate);
    while (ros::ok())
    {
        fetchWeedSrv.request.caller = 1;

        // IF we do get a new weed
        if (fetchWeedClient.call(fetchWeedSrv))
        {
            ROS_INFO("urGovernor -- WEED @ (%.1f,%.1f,%.1f) [cm]",
                    fetchWeedSrv.response.weed.x_cm, fetchWeedSrv.response.weed.y_cm, fetchWeedSrv.response.weed.z_cm);

            // UpRoot this weed!
            // This call will block until the arm is back in it's rest position
            // This call will 

            if (doConstantTracking)
            {
                // This call will do a continual update uproot
                markUprootedSrv.request.success = doConstantTrackingUproot(fetchWeedSrv);
            }
            else
            {
                // This call will do a static uproot
                markUprootedSrv.request.success = doStaticUproot(fetchWeedSrv);
            }

            // Mark this weed as uprooted (or back to ready if not successful)
            markUprootedSrv.request.tracking_id = fetchWeedSrv.response.tracking_id;
            if (!markUprootedClient.call(markUprootedSrv))
            {
                ROS_ERROR("Governor -- Error calling markUprooted Srv (call to tracker_node).");
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
