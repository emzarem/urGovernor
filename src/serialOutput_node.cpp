#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

// Srv and msg types
#include <urGovernor/SerialOutput.h>
using namespace std;

// Parameters to read from configs
std::string serialServiceName;
std::string serialPort;
int serialBaudRate;

// Serial output instance
serial::Serial ser;

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("serial_output_service", serialServiceName)) return false;
    if (!nodeHandle.getParam("serial_port", serialPort)) return false;
    if (!nodeHandle.getParam("serial_baud_rate", serialBaudRate)) return false;

    return true;
}

// Serial Output service (called by controller)
bool serialOutput(urGovernor::SerialOutput::Request &req, urGovernor::SerialOutput::Response &res)
{
    std::string msg = req.command;

    // TODO: Add in some error checking here on the serial port.

    ROS_INFO_STREAM("Writing to serial port: " << msg);
    
    msg += "\n";
    // Send over serial
    ser.write(msg);
    res.status = 0;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serialOutput_node");
    ros::NodeHandle nodeHandle("~");

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for serialOutput.");
        ros::requestShutdown();
    }

    // Setting up serial ...
    try
    {
        ser.setPort(serialPort);
        ser.setBaudrate(serialBaudRate);
	    ser.setBytesize(serial::eightbits);
	    ser.setFlowcontrol(serial::flowcontrol_none);
	    ser.setParity(serial::parity_none);
	    ser.setStopbits(serial::stopbits_one);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO("Serial Port initialized");
    }else{
        return -1;
    }

    // Service to write to serial
    ros::ServiceServer service = nodeHandle.advertiseService(serialServiceName, serialOutput);

    ros::spin();
}

