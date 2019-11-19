#include <ros/ros.h>
#include "urGovernor/Tracker.h"
#include "urVision/weedData.h"


static const Tracker* p_tracker;

//bool fetch_weed(urGovernor::FetchWeed::Request &req, urGovernor::FetchWeed::Response &res)
//{
//
//}

void new_weed_callback(const urVision::weedData::ConstPtr& msg)
{
    ROS_INFO("New weed: x- %d    y- %d     t- %f      size- %d", msg->x_cm, msg->y_cm, msg->time, msg->size_cm);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Tracker_Node");
    ros::NodeHandle n;

    p_tracker = new Tracker();

//    ros::ServiceServer service = n.advertiseService("fetch_weed", fetch_weed);
    ros::Subscriber sub = n.subscribe("new_weeds", 1000, new_weed_callback);

    ros::spin();
    delete p_tracker;
}
