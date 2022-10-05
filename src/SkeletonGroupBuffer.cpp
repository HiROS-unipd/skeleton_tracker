// Internal dependencies
#include "skeleton_tracker/SkeletonGroupBuffer.h"

#include "skeleton_tracker/utils.h"

hiros::skeletons::SkeletonGroupBuffer::SkeletonGroupBuffer(
    const double& dt_epsilon)
    : DT_EPSILON(dt_epsilon) {}

size_t hiros::skeletons::SkeletonGroupBuffer::size() const {
  return buffer_.size();
}

bool hiros::skeletons::SkeletonGroupBuffer::empty() const {
  return buffer_.empty();
}

void hiros::skeletons::SkeletonGroupBuffer::push_back(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  if (!utils::isEmpty(msg)) {
    buffer_.emplace(utils::avgSrcTime(msg), msg);
  }
}

void hiros::skeletons::SkeletonGroupBuffer::pop_front() {
  buffer_.erase(buffer_.begin());
}

void hiros::skeletons::SkeletonGroupBuffer::erase_until_time(
    const rclcpp::Time& time) {
  for (auto it{buffer_.begin()}; it != buffer_.end();) {
    if ((it->first - time).seconds() < DT_EPSILON) {
      it = buffer_.erase(it);
    } else {
      ++it;
    }
  }
}

rclcpp::Time hiros::skeletons::SkeletonGroupBuffer::get_src_time() const {
  return buffer_.begin()->first;
}

std::string hiros::skeletons::SkeletonGroupBuffer::get_src_frame() const {
  return buffer_.begin()->second.skeletons.front().src_frame;
}

const hiros_skeleton_msgs::msg::SkeletonGroup&
hiros::skeletons::SkeletonGroupBuffer::get_skeleton_group() const {
  return buffer_.begin()->second;
}

hiros_skeleton_msgs::msg::SkeletonGroup&
hiros::skeletons::SkeletonGroupBuffer::get_skeleton_group() {
  return buffer_.begin()->second;
}
