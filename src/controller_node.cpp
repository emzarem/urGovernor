#include <ros/ros.h>

// Shared lib
#include "SerialPacket.h"

// For kinematics
#include "deltaRobot.h"

// Srv and msg types
#include <urGovernor/FetchWeed.h>
#include <urVision/weedDataArray.h>
#include <urGovernor/SerialWrite.h>
#include <urGovernor/SerialRead.h>

// Parameters to read from configs
std::string fetchWeedServiceName;
float overallRate;

std::string serialServiceWriteName;
std::string serialServiceReadName;

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
    if (!nodeHandle.getParam("controller_overall_rate", overallRate)) return false;

    if (!nodeHandle.getParam("end_effector_time_s", endEffectorTime)) return false;

    if (!nodeHandle.getParam("soil_offset", soilOffset)) return false;

    if (!nodeHandle.getParam("serial_output_service", serialServiceWriteName)) return false;
    if (!nodeHandle.getParam("serial_input_service", serialServiceReadName)) return false;

    if (!nodeHandle.getParam("serial_timeout_ms", serialTimeoutMs)) return false;
    
    return true;
}

/* 
 * This is the main blocking call to set the arm position to the angles specified 
 */
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle nodeHandle("~");

    int fetchWeedLogs = 0;

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for controller_node.");
        ros::requestShutdown();
    }

    serialWriteClient = nh.serviceClient<urGovernor::SerialWrite>(serialServiceWriteName);
    ros::service::waitForService(serialServiceWriteName);

    serialReadClient = nh.serviceClient<urGovernor::SerialRead>(serialServiceReadName);
    ros::service::waitForService(serialServiceReadName);

    // Subscribe to service from governor
    ros::ServiceClient fetchWeedClient = nh.serviceClient<urGovernor::FetchWeed>(fetchWeedServiceName);
    ros::service::waitForService(fetchWeedServiceName);
    urGovernor::FetchWeed fetchWeedSrv;

    /* Initializing Kinematics */
    // Set tool offset (tool id == 0, x, y, z )
    robot_tool_offset(0, 0, 0, 0);
    // Default deltarobot setup
    deltarobot_setup();

    // ZERO ARM POSITIONS
    if (!actuateArmAngles(0, 0, 0))
    {
        ROS_ERROR("Could not Initialize arm positions.");
        ros::requestShutdown();
    }

    /* 
     * Main loop for controller
     */
    ros::Rate loopRate(overallRate);
    while (ros::ok())
    {
        fetchWeedSrv.request.caller = 1;
        // Call for a new weed
        if (fetchWeedClient.call(fetchWeedSrv))
        {
            /* Create coordinates in the Delta Arm Reference
            *   This conversion requires a 'rotation matrix' 
            *   to be applied to comply with Delta library coordinates.
            *   x' = x*cos(theta) - y*sin(theta)
            *   y' = x*sin(theta) + y*cos(theta)
            * Based on our setup, theta = +60 degrees AND X and Y coordinates are switched
            */
            float x_coord = (float)(fetchWeedSrv.response.weed.y_cm*(0.5) - (fetchWeedSrv.response.weed.x_cm)*(0.866));
            float y_coord = (float)(fetchWeedSrv.response.weed.y_cm*(0.866) + (fetchWeedSrv.response.weed.x_cm)*(0.5));
            // z = 0 IS AT THE GROUND (z = is always positive)
            float z_coord = (float)(fetchWeedSrv.response.weed.z_cm + soilOffset);
            
            /* Calculate angles for Delta arm */
            robot_position(x_coord, y_coord, z_coord); 

            int angle1Deg,angle2Deg,angle3Deg;

            // Get the resulting angles from kinematics
            if (getArmAngles(&angle1Deg, &angle2Deg, &angle3Deg))
            {
                ROS_INFO("Delta for coords (%.2f,%.2f,%.2f) [cm] -> (%i,%i,%i) [degrees]",
                            fetchWeedSrv.response.weed.x_cm, fetchWeedSrv.response.weed.y_cm, fetchWeedSrv.response.weed.z_cm, 
                            angle1Deg, angle2Deg, angle3Deg);

                // // Time the blocking call to actuate arm angles
                // ros::WallTime start_, end_;
                // start_ = ros::WallTime::now();
                
                // Block until we've actuated to these angles
                if (actuateArmAngles(angle1Deg, angle2Deg, angle3Deg))
                {
                    // Wait for end-effectors
                    // TODO: or send a different command?
                    ros::Duration(endEffectorTime).sleep();

                    // RESET ARM POSITIONS
                    if (!actuateArmAngles(0, 0, 0))
                    {
                        ROS_ERROR("Could not Reset arm positions.");
                        ros::requestShutdown();
                    }
                }
                else
                {
                    ROS_ERROR("Could not actuate motors to specified arm angles");
                    ros::requestShutdown();
                }

                // end_ = ros::WallTime::now();
                // double execution_time = (end_ - start_).toNSec() * 1e-6;
                // ROS_INFO_STREAM("Actuation time for weed (ms): " << execution_time);
            }
            else
            {
                ROS_ERROR("Could not get arm angles.");
            }
        }
        else
        {
            if (fetchWeedLogs % logFetchWeedInterval == 1)
            {
                ROS_INFO("Controller -- no weeds are current.");
            }
            fetchWeedLogs++;
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
