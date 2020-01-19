#include <ros/ros.h>

// Shared lib
#include "SerialPacket.h"

// Srv and msg types
#include <urGovernor/SerialOutput.h>

// Parameters to read from configs
std::string serialServiceName;


// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("serial_output_service", serialServiceName)) return false;

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
    ros::ServiceClient serialClient = nh.serviceClient<urGovernor::SerialOutput>(serialServiceName);
    ros::service::waitForService(serialServiceName);
    urGovernor::SerialOutput serialSrv;


    // Main loop
    ros::Rate loopRate(0.2);

    while (ros::ok())
    {
        SerialUtils::CmdMsg msg = {0, -19.0, 1, 1};
        std::vector<char> buff;
        SerialUtils::pack(buff, msg);

        serialSrv.request.command = std::string(buff.begin(), buff.end());

        // Send angles to HAL (via calling the serial client)
        if (!serialClient.call(serialSrv))
        {
            ROS_ERROR("Serial output was NOT successful.");
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
