#include <ros/ros.h>

// Shared lib
#include "SerialPacket.h"

// For kinematics
#include "deltaRobot.h"

// Srv and msg types
#include <urGovernor/SerialWrite.h>
#include <urGovernor/SerialRead.h>
#include <urGovernor/KinematicsTest.h>
#include <urGovernor/MotorConfigTest.h>

// Parameters to read from configs
std::string serialServiceWriteName;
std::string serialServiceReadName;
double endEffectorTime = 0;
int serialTimeoutMs;
float toolOffset = 0;
float soilOffset = 0;

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
  
    if (!nodeHandle.getParam("tool_offset", toolOffset)) return false;
    if (!nodeHandle.getParam("soil_offset", soilOffset)) return false;

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
        .mtr_angles = {(uint32_t)angle1Deg, (uint32_t)angle2Deg, (uint32_t)angle3Deg}
    };
    std::vector<char> buff;
    SerialUtils::pack(buff, msg);

    SerialUtils::CmdMsg ret_msg;

    ROS_INFO("Setting angles: %i %i %i", msg.mtr_angles[0], msg.mtr_angles[1],msg.mtr_angles[2]);

    serialWrite.request.command = std::string(buff.begin(), buff.end());

    // Send angles to HAL (via calling the serial WRITE client)
    if (serialWriteClient.call(serialWrite))
    {
        ros::Rate loopRate( 1.0 / (serialTimeoutMs / 1000.0));
       
        ros::WallTime start_time = ros::WallTime::now();
        double timeout = 2;

        while (ros::ok() && (ros::WallTime::now()- start_time).toSec() < timeout )
        {
            // Wait for arm done
            // This is done by calling the serial READ client
            // This should block until we get a CmdMsg FROM the serial line
            if (serialReadClient.call(serialRead))
            {
                std::vector<char> v(serialRead.response.command.begin(), serialRead.response.command.end());
                
                ret_msg.cmd_success = 0;
                // Unpack response from read
                SerialUtils::unpack(v, ret_msg);

                ROS_INFO("Heres the return msg: %s" , std::string(ret_msg));

                // This should indicate that we are done
                if (ret_msg == msg && ret_msg.cmd_success)
                {
                    ROS_INFO("Motor callback received.");
                    return 1;
                }
            }
            else
            {
                ROS_DEBUG("Timed out waiting for response from Teensy ... retrying ...");
            }
            loopRate.sleep();
        }
        ROS_ERROR("Timed out on response from teensy");
    }
    else
    {
        ROS_ERROR("Serial write to set motors was NOT successful.");
    }
    
    return 0;
}


bool moveToCoords(urGovernor::KinematicsTest::Request &req, urGovernor::KinematicsTest::Response &res) {
    /* Create coordinates in the Delta Arm Reference
    *   This conversion requires a 'rotation matrix' 
    *   to be applied to comply with Delta library coordinates.
    *   x' = x*cos(theta) - y*sin(theta)
    *   y' = x*sin(theta) + y*cos(theta)
    * Based on our setup, theta = +60 degrees AND X and Y coordinates are switched
    */
    float x_coord = (float)(req.y_coord*(0.5) - (req.x_coord)*(0.866));
    float y_coord = (float)(req.y_coord*(0.866) + (req.x_coord)*(0.5));
    float z_coord = (float)req.z_coord + soilOffset;    // z = 0 IS AT THE GROUND (z = is always positive)
    
    /* Calculate angles for Delta arm */
    robot_position(x_coord, y_coord, z_coord); 

    int angle1Deg,angle2Deg,angle3Deg;

    res.success = false; // default no success

    // Get the resulting angles from kinematics
    if (getArmAngles(&angle1Deg, &angle2Deg, &angle3Deg))
    {
        ROS_INFO("Delta for coords (%.2f,%.2f,%.2f) [cm] -> (%i,%i,%i) [degrees]",
                    (float)req.x_coord, (float)req.y_coord, (float)req.z_coord, 
                    (uint32_t)angle1Deg, (uint32_t)angle2Deg, (uint32_t)angle3Deg);
        
        if (angle1Deg < 0 || angle2Deg < 0 || angle3Deg < 0)
        {
            ROS_ERROR("Got negative angles ...");
            return false;
        }

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

bool moveToAngles(urGovernor::KinematicsTest::Request &req, urGovernor::KinematicsTest::Response &res) {
    res.success = false; // default no success

    int angle1Deg = (int)req.x_coord;
    int angle2Deg = (int)req.y_coord;
    int angle3Deg = (int)req.z_coord;

    ROS_INFO("Delta arm actuating: (%i,%i,%i) [degrees]",
                (float)req.x_coord, (float)req.y_coord, (float)req.z_coord, 
                angle1Deg, angle2Deg, angle3Deg);
    
    // Block until we've actuated to these angles
    if (actuateArmAngles(angle1Deg, angle2Deg, angle3Deg))
    {
        res.success = true;
        return true;          
    }
    else
    {
        ROS_ERROR("Could not actuate motors to specified arm angles");
        // ros::requestShutdown();
        return false;
    }

    return true;
}

bool adjustSpeed(urGovernor::MotorConfigTest::Request &req, urGovernor::MotorConfigTest::Response &res) {
    urGovernor::SerialWrite serialWrite;
    urGovernor::SerialRead serialRead;   
    SerialUtils::CmdMsg msg = {
        .cmd_type = SerialUtils::CMDTYPE_CONFIG,
    };
    msg.mtr_speed_deg_s = req.speed;
    msg.mtr_accel_deg_s_s = req.accel;

    std::vector<char> buff;
    SerialUtils::pack(buff, msg);

    SerialUtils::CmdMsg ret_msg;

    ROS_INFO("Setting config: %d %d", msg.mtr_speed_deg_s, msg.mtr_accel_deg_s_s);

    serialWrite.request.command = std::string(buff.begin(), buff.end());

    res.success = false;
    // Send angles to HAL (via calling the serial WRITE client)
    if (serialWriteClient.call(serialWrite))
    {
        ros::Rate loopRate( 1.0 / (serialTimeoutMs / 1000.0));
        ros::WallTime start_time = ros::WallTime::now();
        double timeout = 2;

        while (ros::ok() && (ros::WallTime::now()- start_time).toSec() < timeout )
        {
            
            if (serialReadClient.call(serialRead))
            {
                std::vector<char> v(serialRead.response.command.begin(), serialRead.response.command.end());
                
                ret_msg.cmd_success = 0;
                // Unpack response from read
                SerialUtils::unpack(v, ret_msg);

                ROS_INFO("Heres the return msg: %s" , std::string(ret_msg));

                // This should indicate that we are done
                if (ret_msg == msg && ret_msg.cmd_success)
                {
                    ROS_INFO("Motor callback received.");
                    res.success = true;
                    return true;
                }
            }
            else
            {
                ROS_DEBUG("Timed out waiting for response from Teensy ... retrying ...");
            }
            loopRate.sleep();
        }
        ROS_ERROR("Timed out on response from teensy");
    }
    else
    {
        ROS_ERROR("Serial write to set motors was NOT successful.");
    }
    
    return false;
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
    robot_tool_offset(0, 0, 0, -(toolOffset));
    // Default deltarobot setup
    deltarobot_setup();

    ros::ServiceServer moveCoordService = nodeHandle.advertiseService("move_to_coords", moveToCoords);
    ros::ServiceServer moveAngleService = nodeHandle.advertiseService("move_to_angles", moveToAngles);
    ros::ServiceServer configService = nodeHandle.advertiseService("mtr_config", adjustSpeed);

    ros::spin();

    return 0;
}
