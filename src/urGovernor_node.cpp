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
int minValidFrames;


static inline Object weed_to_object(urVision::weedData& weed)
{
    /* all values are floats */
    return {(float)weed.x_cm, (float)weed.y_cm, (float)weed.z_cm, (float)weed.size_cm};
}

static void object_to_weed(Object& obj, urVision::weedData& weed)
{
    weed.x_cm = obj.x;
    weed.y_cm = obj.y;
    weed.z_cm = obj.z;
    weed.size_cm = obj.size;
}

/* Fetch weed service (called by controller):
 *      Returns
 * 
 * 
 */
bool fetch_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
{
    Object top_valid_obj;
    if (p_tracker->topValid(top_valid_obj))
    {
        /* Set response data to include weed fetched from the top */
        object_to_weed(top_valid_obj, res.weed);
        return true;
    }
    else
    {
        ROS_DEBUG("fetch_weed_service: No current weeds are valid (yet).");
        return false;
    }
}

void new_weed_callback(const urVision::weedDataArray::ConstPtr& msg)
{
    ROS_DEBUG("Weed array received.");
    
    std::vector<Object> new_objs;

    for (auto weed : msg->weeds)
    {
        ROS_DEBUG("\tNew weed: x- %f    y- %f      z- %f      size- %f", weed.x_cm, weed.y_cm, weed.z_cm, weed.size_cm);
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
    if (!nodeHandle.getParam("min_valid_frames", minValidFrames)) return false;
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

    p_tracker = new ObjectTracker(distanceTolerance, maxDisappearedFrms, minValidFrames);

    // Subscriber to the weed publisher (from urVision)
    ros::Subscriber sub = nodeHandle.subscribe(weedPublisherName, 1000, new_weed_callback);

    // Service to provide to controller
    ros::ServiceServer service = nodeHandle.advertiseService(fetchWeedServiceName, fetch_weed);

    ros::spin();
    delete p_tracker;
}
