#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

// Shared lib
#include "SerialPacket.h"

// Srv and msg types
#include <urGovernor/SerialWrite.h>
#include <urGovernor/SerialRead.h>
using namespace std;

// Parameters to read from configs
std::string serialServiceWriteName;
std::string serialServiceReadName;

// Serial Write service (called by controller to send motor angles)
bool serialWrite(urGovernor::SerialWrite::Request &req, urGovernor::SerialWrite::Response &res)
{
    std::vector<char> v(req.command.begin(), req.command.end());
    SerialUtils::CmdMsg msg;
    // Unpack response from read
    SerialUtils::unpack(v, msg);

    ROS_DEBUG_STREAM("Writing to serial: " << std::endl << std::string(msg));

    return true;
}

// Serial Read service (called by controller to sychronize end of motor movement)
bool serialRead(urGovernor::SerialRead::Request &req, urGovernor::SerialRead::Response &res)
{
    SerialUtils::CmdMsg msg = { .cmd_type = SerialUtils::CMDTYPE_MTRS };
    msg.cmd_success = 1;
    std::vector<char> buff;

    SerialUtils::pack(buff, msg);
    res.command = std::string(buff.begin(), buff.end());

    // Put an artificial sleep in here
    ros::Duration(0.5).sleep();

    ROS_DEBUG_STREAM("Reading from serial: " << std::endl << std::string(msg));

    return true;
}

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("serial_output_service", serialServiceWriteName)) return false;
    if (!nodeHandle.getParam("serial_input_service", serialServiceReadName)) return false;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serialOutput_node");
    ros::NodeHandle nodeHandle("~");

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for serialStub_node.");
        ros::requestShutdown();
    }

    // Service to write to serial
    ros::ServiceServer writeService = nodeHandle.advertiseService(serialServiceWriteName, serialWrite);

    // Service to read from serial
    ros::ServiceServer readService = nodeHandle.advertiseService(serialServiceReadName, serialRead);

    ros::spin();
}

