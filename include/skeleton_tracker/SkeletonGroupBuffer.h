#ifndef hiros_skeleton_tracker_SkeletonGroupBuffer_h
#define hiros_skeleton_tracker_SkeletonGroupBuffer_h

// Standard dependencies
#include <map>

// ROS dependencies
#include "ros/time.h"

// Custom external dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"

namespace hiros {
  namespace track {

    class SkeletonGroupBuffer
    {
    public:
      SkeletonGroupBuffer(const double& t_dt_epsilon = 0.001);

      size_t size() const;
      bool empty() const;

      void push_back(const hiros_skeleton_msgs::SkeletonGroup& t_msg);
      void pop_front();
      void erase_until_time(const ros::Time& t_time);

      ros::Time get_src_time() const;
      std::string get_src_frame() const;
      const hiros_skeleton_msgs::SkeletonGroup& get_skeleton_group() const;
      hiros_skeleton_msgs::SkeletonGroup& get_skeleton_group();

    private:
      const double DT_EPSILON{0.001};
      std::map<ros::Time, hiros_skeleton_msgs::SkeletonGroup> m_buffer{};
    };

  } // namespace track
} // namespace hiros

#endif
