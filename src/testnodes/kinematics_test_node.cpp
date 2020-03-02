#include <ros/ros.h>

// Shared lib
#include "SerialPacket.h"

// For kinematics
#include "deltaRobot.h"

// Srv and msg types
#include <urGovernor/SerialWrite.h>
#include <urGovernor/SerialRead.h>
#include <urGovernor/KinematicsTest.h>

// Parameters to read from configs
std::string serialServiceWriteName;
std::string serialServiceReadName;
double endEffectorTime = 0;
int serialTimeoutMs;

// Connections to Serial interface services
ros::ServiceClient serialWriteClient;
ros::ServiceClient serialReadClient;


// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("serial_output_service", serialServiceWriteName)) return false;
    if (!nodeHandle.getParam("serial_input_service", serialServiceReadName)) return false;
    if (!nodeHandle.getParam("end_effector_time_s", endEffectorTime)) return false;
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
    SerialUtils::CmdMsg msg = {
        .cmd_type = SerialUtils::CMDTYPE_MTRS,
        .is_relative = 0,
        .mtr_angles = {(float)angle1Deg, (float)angle2Deg, (float)angle3Deg}
    };
    std::vector<char> buff;
    SerialUtils::pack(buff, msg);

    ROS_INFO("Setting angles: %d %d %d", msg.mtr_angles[0], msg.mtr_angles[1],msg.mtr_angles[2]);

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
                
                msg.cmd_success = 0;
                // Unpack response from read
                SerialUtils::unpack(v, msg);

                ROS_INFO("Heres the return msg: %s" , std::string(msg));

                // This should indicate that we are done
                if (msg.cmd_type == SerialUtils::CMDTYPE_RESP && msg.cmd_success)
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


bool moveArm(urGovernor::KinematicsTest::Request &req, urGovernor::KinematicsTest::Response &res) {
    /* Create coordinates in the Delta Arm Reference
    *   This conversion requires a 'rotation matrix' 
    *   to be applied to comply with Delta library coordinates.
    *   x' = x*cos(theta) - y*sin(theta)
    *   y' = x*sin(theta) + y*cos(theta)
    * Based on our setup, theta = +60 degrees AND X and Y coordinates are switched
    */
    float x_coord = (float)(req.y_coord*(0.5) - (req.x_coord)*(0.866));
    float y_coord = (float)(req.y_coord*(0.866) + (req.x_coord)*(0.5));
    float z_coord = (float)req.z_coord;    // z = 0 IS AT THE GROUND (z = is always positive)
    
    /* Calculate angles for Delta arm */
    robot_position(x_coord, y_coord, z_coord); 

    int angle1Deg,angle2Deg,angle3Deg;

    res.success = false; // default no success

    // Get the resulting angles from kinematics
    if (getArmAngles(&angle1Deg, &angle2Deg, &angle3Deg))
    {
        ROS_INFO("Delta for coords (%.2f,%.2f,%.2f) [cm] -> (%i,%i,%i) [degrees]",
                    (float)req.x_coord, (float)req.y_coord, (float)req.z_coord, 
                    angle1Deg, angle2Deg, angle3Deg);

        // // Time the blocking call to actuate arm angles
        // ros::WallTime start_, end_;
        // start_ = ros::WallTime::now();
        
        // Block until we've actuated to these angles
        if (actuateArmAngles(angle1Deg, angle2Deg, angle3Deg))
        {
            res.success = true;
            return true;          
        }
        else
        {
            ROS_ERROR("Could not actuate motors to specified arm angles");
            ros::requestShutdown();
            return false;
        }

        // end_ = ros::WallTime::now();
        // double execution_time = (end_ - start_).toNSec() * 1e-6;
        // ROS_INFO_STREAM("Actuation time for weed (ms): " << execution_time);
    }
    else
    {
        ROS_ERROR("Could not get arm angles.");
    }

    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematics_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nodeHandle("~");

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for kinematics_test_node.");
        ros::requestShutdown();
    }

    serialWriteClient = nh.serviceClient<urGovernor::SerialWrite>(serialServiceWriteName);
    ros::service::waitForService(serialServiceWriteName);

    serialReadClient = nh.serviceClient<urGovernor::SerialRead>(serialServiceReadName);
    ros::service::waitForService(serialServiceReadName);

    /* Initializing Kinematics */
    // Set tool offset (tool id == 0, x, y, z )
    robot_tool_offset(0, 0, 0, 0);
    // Default deltarobot setup
    deltarobot_setup();

    ros::ServiceServer moveService = nodeHandle.advertiseService("move_to_coords", moveArm);

    ros::spin();

    return 0;
}
