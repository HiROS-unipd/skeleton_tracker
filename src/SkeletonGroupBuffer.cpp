// Internal dependencies
#include "skeleton_tracker/SkeletonGroupBuffer.h"
#include "skeleton_tracker/utils.h"

hiros::track::SkeletonGroupBuffer::SkeletonGroupBuffer(const double& t_dt_epsilon)
  : DT_EPSILON(t_dt_epsilon)
{}

size_t hiros::track::SkeletonGroupBuffer::size() const
{
  return m_buffer.size();
}

bool hiros::track::SkeletonGroupBuffer::empty() const
{
  return m_buffer.empty();
}

void hiros::track::SkeletonGroupBuffer::push_back(const hiros_skeleton_msgs::SkeletonGroup& t_msg)
{
  if (!utils::isEmpty(t_msg)) {
    m_buffer.emplace(utils::avgSrcTime(t_msg), t_msg);
  }
}

void hiros::track::SkeletonGroupBuffer::pop_front()
{
  m_buffer.erase(m_buffer.begin());
}

void hiros::track::SkeletonGroupBuffer::erase_until_time(const ros::Time& t_time)
{
  std::erase_if(m_buffer, [&](const auto& e) { return ((e.first - t_time).toSec() < DT_EPSILON); });
}

ros::Time hiros::track::SkeletonGroupBuffer::get_src_time() const
{
  return m_buffer.begin()->first;
}

std::string hiros::track::SkeletonGroupBuffer::get_src_frame() const
{
  return m_buffer.begin()->second.skeletons.front().src_frame;
}

const hiros_skeleton_msgs::SkeletonGroup& hiros::track::SkeletonGroupBuffer::get_skeleton_group() const
{
  return m_buffer.begin()->second;
}

hiros_skeleton_msgs::SkeletonGroup& hiros::track::SkeletonGroupBuffer::get_skeleton_group()
{
  return m_buffer.begin()->second;
}
