#ifndef hiros_skeleton_tracker_SkeletonGroupBuffer_h
#define hiros_skeleton_tracker_SkeletonGroupBuffer_h

// Standard dependencies
#include <map>

// ROS dependencies
#include <rclcpp/time.hpp>

// Custom external dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"

namespace hiros {
namespace skeletons {

class SkeletonGroupBuffer {
 public:
  SkeletonGroupBuffer(const double& dt_epsilon = 0.001);

  size_t size() const;
  bool empty() const;

  void push_back(const hiros_skeleton_msgs::msg::SkeletonGroup& msg);
  void pop_front();
  void erase_until_time(const rclcpp::Time& time);

  rclcpp::Time get_src_time() const;
  std::string get_src_frame() const;
  const hiros_skeleton_msgs::msg::SkeletonGroup& get_skeleton_group() const;
  hiros_skeleton_msgs::msg::SkeletonGroup& get_skeleton_group();

 private:
  const double DT_EPSILON{};
  std::map<rclcpp::Time, hiros_skeleton_msgs::msg::SkeletonGroup> buffer_{};
};

}  // namespace skeletons
}  // namespace hiros

#endif
