// ROS
#include <ros/ros.h>

// Internal dependencies
#include "skeleton_tracker/OrientationSkeletonTracker.h"

int main(int argc, char* argv[])
{
  std::string node_name = "hiros_ost";
  ros::init(argc, argv, node_name);

  hiros::track::OrientationSkeletonTracker tracker;
  tracker.start();

  ros::spin();

  return 0;
}
