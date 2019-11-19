#include <ros/ros.h>
#include "urVision/weedData.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urGovernor");
  ros::NodeHandle nodeHandle("~");

  ros::spin();
  return 0;
}
