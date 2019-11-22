#include <ros/ros.h>

#include "urGovernor/ObjectTracker.h"
#include "urGovernor/FetchWeed.h"

#include "urVision/weedData.h"


static const ObjectTracker* p_tracker;

bool fetch_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
{
    ROS_INFO("Caller - %x", req.caller);
    res.weed.x_cm = 1;
}

void new_weed_callback(const urVision::weedData::ConstPtr& msg)
{
    ROS_INFO("New weed: x- %d    y- %d     t- %f      size- %d", msg->x_cm, msg->y_cm, msg->time, msg->size_cm);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectTracker_Node");
    ros::NodeHandle n;

    p_tracker = new ObjectTracker();

    ros::ServiceServer service = n.advertiseService("fetch_weed", fetch_weed);
    ros::Subscriber sub = n.subscribe("new_weeds", 1000, new_weed_callback);

    ros::spin();
    delete p_tracker;
}
