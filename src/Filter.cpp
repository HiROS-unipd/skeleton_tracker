// ROS dependencies
#include <tf2/LinearMath/Matrix3x3.h>

// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/Filter.h"

void hiros::track::KinematicStateFilter::filter(hiros::skeletons::types::KinematicState& t_state,
                                                const double& t_time,
                                                const double& t_cutoff)
{
  auto prev_time = m_last_time;
  auto prev_state = m_last_state;

  computePose(t_state, t_time, t_cutoff);
  computeVelocity(t_state, prev_state, prev_time, t_cutoff);
  computeAcceleration(t_state, prev_state, prev_time, t_cutoff);
}

void hiros::track::KinematicStateFilter::computePose(hiros::skeletons::types::KinematicState& t_state,
                                                     const double& t_time,
                                                     const double& t_cutoff)
{
  // init
  if (std::isnan(m_last_time)) {
    m_last_time = t_time;
    m_last_state = t_state;

    if (skeletons::utils::isNaN(m_last_state.velocity.linear)) {
      m_last_state.velocity.linear = skeletons::types::Vector3(0, 0, 0);
    }
    if (skeletons::utils::isNaN(m_last_state.velocity.angular)) {
      m_last_state.velocity.angular = skeletons::types::Vector3(0, 0, 0);
    }

    if (skeletons::utils::isNaN(m_last_state.acceleration.linear)) {
      m_last_state.velocity.linear = skeletons::types::Vector3(0, 0, 0);
    }
    if (skeletons::utils::isNaN(m_last_state.acceleration.angular)) {
      m_last_state.velocity.angular = skeletons::types::Vector3(0, 0, 0);
    }

    return;
  }

  // filter
  double weight = std::max(0., std::min((t_time - m_last_time) * t_cutoff, 1.));
  m_last_state.pose.position = m_last_state.pose.position.lerp(t_state.pose.position, weight);
  m_last_state.pose.orientation = m_last_state.pose.orientation.slerp(t_state.pose.orientation, weight);

  t_state.pose = m_last_state.pose;
  m_last_time = t_time;
}

void hiros::track::KinematicStateFilter::computeVelocity(hiros::skeletons::types::KinematicState& t_state,
                                                         const hiros::skeletons::types::KinematicState& t_prev_state,
                                                         const double& t_prev_time,
                                                         const double& t_cutoff)
{
  // init
  if (skeletons::utils::isNaN(t_prev_state.velocity)) {
    return;
  }

  // compute velocities
  double delta_roll, delta_pitch, delta_yaw;
  tf2::Vector3 ang_vel;
  double dt = m_last_time - t_prev_time;

  if (dt <= 0) {
    return;
  }

  auto lin_vel = (m_last_state.pose.position - t_prev_state.pose.position) / dt;

  tf2::Matrix3x3 m(m_last_state.pose.orientation * t_prev_state.pose.orientation.inverse());
  m.getEulerYPR(delta_yaw, delta_pitch, delta_roll);

  ang_vel.setX(delta_roll / dt);
  ang_vel.setY(delta_pitch / dt);
  ang_vel.setZ(delta_yaw / dt);

  // filter
  double weight = std::max(0., std::min(dt * t_cutoff, 1.));
  m_last_state.velocity.linear = m_last_state.velocity.linear.lerp(lin_vel, weight);
  m_last_state.velocity.angular = m_last_state.velocity.angular.lerp(ang_vel, weight);

  t_state.velocity = m_last_state.velocity;
}

void hiros::track::KinematicStateFilter::computeAcceleration(
  hiros::skeletons::types::KinematicState& t_state,
  const hiros::skeletons::types::KinematicState& t_prev_state,
  const double& t_prev_time,
  const double& t_cutoff)
{
  // init
  if (skeletons::utils::isNaN(t_prev_state.acceleration)) {
    return;
  }

  // compute accelerations
  double dt = m_last_time - t_prev_time;

  if (dt <= 0) {
    return;
  }

  auto lin_acc = (m_last_state.velocity.linear - t_prev_state.velocity.linear) / dt;
  auto ang_acc = (m_last_state.velocity.angular - t_prev_state.velocity.angular) / dt;

  // filter
  double weight = std::max(0., std::min(dt * t_cutoff, 1.));
  m_last_state.acceleration.linear = m_last_state.acceleration.linear.lerp(lin_acc, weight);
  m_last_state.acceleration.angular = m_last_state.acceleration.angular.lerp(ang_acc, weight);

  t_state.acceleration = m_last_state.acceleration;
}

hiros::track::Filter::Filter(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff)
{
  init(t_skeleton, t_cutoff);
}

void hiros::track::Filter::init(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff)
{
  if (!m_initialized) {
    for (auto& mk : t_skeleton.markers) {
      m_marker_filters[mk.id] = KinematicStateFilter();
      m_marker_filters[mk.id].filter(mk.center, t_skeleton.src_time, t_cutoff);
    }

    for (auto& lk : t_skeleton.links) {
      m_link_filters[lk.id] = KinematicStateFilter();
      m_link_filters[lk.id].filter(lk.center, t_skeleton.src_time, t_cutoff);
    }

    m_cutoff = t_cutoff;
    m_initialized = true;
  }
}

void hiros::track::Filter::updateMarkerFilters(hiros::skeletons::types::Skeleton& t_skeleton)
{
  // erase empty markers
  std::erase_if(m_marker_filters, [&](const auto& pair) { return !t_skeleton.hasMarker(pair.first); });

  for (auto& mk : t_skeleton.markers) {
    if (m_marker_filters.count(mk.id) == 0) {
      // new marker
      m_marker_filters[mk.id] = KinematicStateFilter();
    }
    m_marker_filters[mk.id].filter(mk.center, t_skeleton.src_time, m_cutoff);
  }
}

void hiros::track::Filter::updateLinkFilters(hiros::skeletons::types::Skeleton& t_skeleton)
{
  // erase empty links
  std::erase_if(m_link_filters, [&](const auto& pair) { return !t_skeleton.hasLink(pair.first); });

  for (auto& lk : t_skeleton.links) {
    if (m_link_filters.count(lk.id) == 0) {
      // new link
      m_link_filters[lk.id] = KinematicStateFilter();
    }
    m_link_filters[lk.id].filter(lk.center, t_skeleton.src_time, m_cutoff);
  }
}

void hiros::track::Filter::filter(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff)
{
  if (!m_initialized) {
    if (t_cutoff < 0 || std::isnan(t_cutoff)) {
      std::cerr << "hiros::track::Filter Warning: cutoff out of range. Must be in ]0, +inf[" << std::endl;
      return;
    }
    init(t_skeleton, t_cutoff);
  }

  updateMarkerFilters(t_skeleton);
  updateLinkFilters(t_skeleton);
}

void hiros::track::Filter::filter(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff) const
{
  auto tmp = *this;
  tmp.filter(t_skeleton, t_cutoff);
}

void hiros::track::Filter::updVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff)
{
  auto original_skel = t_skeleton;
  filter(t_skeleton, t_cutoff);

  // reset marker poses to the values before filtering
  for (auto& mk : t_skeleton.markers) {
    mk.center.pose = original_skel.getMarker(mk.id).center.pose;
  }

  // reset link poses to the values before filtering
  for (auto& lk : t_skeleton.links) {
    lk.center.pose = original_skel.getLink(lk.id).center.pose;
  }
}

void hiros::track::Filter::updVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff) const
{
  auto tmp = *this;
  tmp.updVelAndAcc(t_skeleton, t_cutoff);
}
