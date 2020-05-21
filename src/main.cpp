// ROS
#include <ros/ros.h>

// Internal dependencies
#include "skeleton_tracker/Tracker.h"

int main(int argc, char* argv[])
{
  std::string node_name = "hiros_opt";
  ros::init(argc, argv, node_name);

  hiros::track::Tracker tracker;
  tracker.start();

  ros::spin();

  return 0;
}
