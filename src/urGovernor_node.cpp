#include <ros/ros.h>
#include "urGovernor/RosPackageTemplate.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urGovernor");
  ros::NodeHandle nodeHandle("~");

  urGovernor::RosPackageTemplate rosPackageTemplate(nodeHandle);

  ros::spin();
  return 0;
}
