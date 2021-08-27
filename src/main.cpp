// ROS
#include <ros/ros.h>

// Internal dependencies
#include "skeleton_tracker/SkeletonTracker.h"

int main(int argc, char* argv[])
{
  std::string node_name = "hiros_mst";
  ros::init(argc, argv, node_name);

  hiros::track::SkeletonTracker st;
  st.start();

  ros::spin();

  return 0;
}
