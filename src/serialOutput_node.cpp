#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

// Srv and msg types
#include <urGovernor/SerialWrite.h>
#include <urGovernor/SerialRead.h>
using namespace std;

// Parameters to read from configs
std::string serialServiceWriteName;
std::string serialServiceReadName;
std::string serialPort;
int serialBaudRate;
int serialTimeoutMs;

// Serial output instance
serial::Serial ser;

// Serial Write service (called by controller to send motor angles)
bool serialWrite(urGovernor::SerialWrite::Request &req, urGovernor::SerialWrite::Response &res)
{
    std::string msg = req.command;

    // TODO: Add in some error checking here on the serial port.
    ROS_INFO_STREAM("Writing to serial port: " << msg);
    
    // Send over serial
    ser.write(msg);
    res.status = 0;

    return true;
}

// Serial Read service (called by controller to sychronize end of motor movement)
bool serialRead(urGovernor::SerialRead::Request &req, urGovernor::SerialRead::Response &res)
{
    // This should block until we read a line (by default this is delimited by '\n')
    res.command = (std::string)ser.readline();

    ROS_INFO_STREAM("Read from serial port: " << std::string(res.command));

    return true;
}

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("serial_output_service", serialServiceWriteName)) return false;
    if (!nodeHandle.getParam("serial_input_service", serialServiceReadName)) return false;
    
    if (!nodeHandle.getParam("serial_port", serialPort)) return false;
    if (!nodeHandle.getParam("serial_baud_rate", serialBaudRate)) return false;

    if (!nodeHandle.getParam("serial_timeout_ms", serialTimeoutMs)) return false;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serialOutput_node");
    ros::NodeHandle nodeHandle("~");

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for serialOutput_node.");
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
        serial::Timeout to = serial::Timeout::simpleTimeout(serialTimeoutMs);
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
    ros::ServiceServer writeService = nodeHandle.advertiseService(serialServiceWriteName, serialWrite);

    // Service to read from serial
    ros::ServiceServer readService = nodeHandle.advertiseService(serialServiceReadName, serialRead);

    ros::spin();
}

