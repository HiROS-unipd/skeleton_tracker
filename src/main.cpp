// ROS
#include <ros/ros.h>

// Internal dependencies
#include "skeleton_tracker/SkeletonTracker.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "hiros_skeleton_tracker");

  hiros::track::SkeletonTracker st;
  st.start();

  return 0;
}
