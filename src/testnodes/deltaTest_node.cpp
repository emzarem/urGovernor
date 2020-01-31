#include <ros/ros.h>

#include "urVision/ObjectTracker.h"

// Srv and msg types
#include <urGovernor/FetchWeed.h>
#include <urVision/weedDataArray.h>

#include <vector>

// Parameters to read from configs
std::string fetchWeedServiceName;

static int testIndex = 0;

// (x,y,size) in cm. Size should be set to 0,not used
Object tests[] = {
    {0,0,0,0},
    {19,19,0,0},
    {-19,-19,0,0},
    {19,0,0,0},
    {0,-19,0,0},
    {10,10,0,0},
    {-5,10,0,0},
    {-10,5,0,0},
};

static void object_to_weed(Object& obj, urVision::weedData& weed)
{
    weed.x_cm = obj.x;
    weed.y_cm = obj.y;
    weed.z_cm = obj.z;
    weed.size_cm = obj.size;
}

/* This is a test implementation of the fetch weed service */
// Fetch weed service (called by governor)
bool fetch_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
{
    /* Increment test index */
    if (testIndex >= sizeof(tests)/sizeof(Object))
    {
        ROS_INFO("fetch_weed_service: No current weeds available");
        return false;
    }
    else
    {
        object_to_weed(tests[testIndex], res.weed);
        testIndex++;
        return true;
    }
}

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deltaTest_node");
    ros::NodeHandle nodeHandle("~");

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for deltaTest.");
        ros::requestShutdown();
    }

    // Service to provide to governor
    ros::ServiceServer service = nodeHandle.advertiseService(fetchWeedServiceName, fetch_weed);

    ros::spin();
}
