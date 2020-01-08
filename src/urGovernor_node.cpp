#include <ros/ros.h>

#include "urGovernor/ObjectTracker.h"

// Srv and msg types
#include <urGovernor/FetchWeed.h>
#include <urVision/weedDataArray.h>

#include <vector>

static ObjectTracker* p_tracker;

// Parameters to read from configs
std::string weedPublisherName;
std::string fetchWeedServiceName;
Distance distanceTolerance;
int maxDisappearedFrms;

static inline Object weed_to_object(urVision::weedData& weed)
{
    return {(int32_t)weed.x_cm, (int32_t)weed.y_cm, (int32_t)weed.size_cm};
}

static void object_to_weed(Object& obj, urVision::weedData& weed)
{
    weed.x_cm = obj.x;
    weed.y_cm = obj.y;
    weed.size_cm = obj.z;
}

// Fetch weed service (called by controller)
bool fetch_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
{
    Object top_obj;
    if (!p_tracker->top(top_obj))
    {
        ROS_INFO("fetch_weed_service: No current weeds available");
        return false;
    }
    else
    {
        object_to_weed(top_obj, res.weed);
        return true;
    }
}

void new_weed_callback(const urVision::weedDataArray::ConstPtr& msg)
{
    ROS_DEBUG("Weed array received.");
    
    std::vector<Object> new_objs;

    for (auto weed : msg->weeds)
    {
        /* x, y are msg::Int32, size is a msg::Float64 */
        ROS_DEBUG("\tNew weed: x- %i    y- %i      size- %i", weed.x_cm, weed.y_cm, (int32_t)weed.size_cm);
        new_objs.push_back(weed_to_object(weed));
    }

    p_tracker->update(new_objs);
}

// General parameters for this node
bool readGeneralParameters(ros::NodeHandle nodeHandle)
{
    if (!nodeHandle.getParam("weed_data_publisher", weedPublisherName)) return false;
    if (!nodeHandle.getParam("fetch_weed_service", fetchWeedServiceName)) return false;

    if (!nodeHandle.getParam("max_disappeared_frames", maxDisappearedFrms)) return false;
    if (!nodeHandle.getParam("distance_tolerance", distanceTolerance)) return false;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "urGovernor_node");
    ros::NodeHandle nodeHandle("~");

    if (!readGeneralParameters(nodeHandle))
    {
        ROS_ERROR("Could not read general parameters for urGovernor.");
        ros::requestShutdown();
    }

    p_tracker = new ObjectTracker(distanceTolerance, maxDisappearedFrms);

    // Subscriber to the weed publisher (from urVision)
    ros::Subscriber sub = nodeHandle.subscribe(weedPublisherName, 1000, new_weed_callback);

    // Service to provide to controller
    ros::ServiceServer service = nodeHandle.advertiseService(fetchWeedServiceName, fetch_weed);

    ros::spin();
    delete p_tracker;
}
