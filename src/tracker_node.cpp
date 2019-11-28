#include <ros/ros.h>

#include "urGovernor/ObjectTracker.h"
#include "urGovernor/FetchWeed.h"

#include "urVision/weedDataArray.h"

#include <vector>

static ObjectTracker* p_tracker;

static inline Object weed_to_object(urVision::weedData& weed)
{
    return {weed.x_cm, weed.y_cm, weed.size_cm};
}

static urVision::weedData object_to_weed(Object& obj)
{
    urVision::weedData weed;
    weed.x_cm = obj.x;
    weed.y_cm = obj.y;
    weed.size_cm = obj.z;
    return weed;
}

bool fetch_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
{
    ROS_INFO("Caller - %x", req.caller);
    Object top_obj;
    p_tracker->top(top_obj);
    res.weed = object_to_weed(top_obj);
}

void new_weed_callback(const urVision::weedDataArray::ConstPtr& msg)
{
    ROS_INFO("Weed array received.");
    
    std::vector<Object> new_objs;

    for (auto weed : msg->weeds)
    {
        ROS_INFO("New weed: x- %d    y- %d     t- %f      size- %d \n", weed.x_cm, weed.y_cm, weed.time, weed.size_cm);
        new_objs.push_back(weed_to_object(weed));
    }

    p_tracker->update(new_objs);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectTracker_Node");
    ros::NodeHandle n;

    p_tracker = new ObjectTracker();

    ros::ServiceServer service = n.advertiseService("fetch_weed", fetch_weed);
    ros::Subscriber sub = n.subscribe("/vision_target/weed_data", 1000, new_weed_callback);

    ros::spin();
    delete p_tracker;
}
