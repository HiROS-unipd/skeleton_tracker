// ROS dependencies
#include <tf2/LinearMath/Matrix3x3.h>

// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/Filter.h"

void hiros::track::MarkerFilter::filter(hiros::skeletons::types::Point& t_point,
                                        const double& t_time,
                                        const double& t_cutoff)
{
  auto prev_position = m_last_position;
  auto prev_time = m_last_time;

  computePosition(t_point, t_time, t_cutoff);
  computeVelocity(t_point, prev_position, prev_time, t_cutoff);
}

void hiros::track::MarkerFilter::computePosition(hiros::skeletons::types::Point& t_point,
                                                 const double& t_time,
                                                 const double& t_cutoff)
{
  // init
  if (std::isnan(m_last_time)) {
    m_last_time = t_time;
    m_last_position = t_point.position;
    m_last_velocity = std::isnan(t_point.velocity.x()) ? tf2::Vector3(0, 0, 0) : t_point.velocity;
    return;
  }

  // filter
  double weight = std::max(0., std::min((t_time - m_last_time) * t_cutoff, 1.));
  m_last_position = m_last_position.lerp(t_point.position, weight);
  t_point.position = m_last_position;
  m_last_time = t_time;
}

void hiros::track::MarkerFilter::computeVelocity(hiros::skeletons::types::Point& t_point,
                                                 const tf2::Vector3& t_prev_position,
                                                 const double& t_prev_time,
                                                 const double& t_cutoff)
{
  // init
  if (std::isnan(t_prev_time)) {
    t_point.velocity = tf2::Vector3(0, 0, 0);
    return;
  }

  // compute linear velocities
  double dt = m_last_time - t_prev_time;
  auto curr_vel = (m_last_position - t_prev_position) / dt;

  // filter
  double weight = std::max(0., std::min(dt * t_cutoff, 1.));

  m_last_velocity = m_last_velocity.lerp(curr_vel, weight);
  t_point.velocity = m_last_velocity;
}

void hiros::track::OrientationFilter::filter(hiros::skeletons::types::MIMU& t_mimu,
                                             const double& t_time,
                                             const double& t_cutoff)
{
  auto prev_orientation = m_last_orientation;
  auto prev_time = m_last_time;

  computeOrientation(t_mimu, t_time, t_cutoff);
  computeVelocity(t_mimu, prev_orientation, prev_time, t_cutoff);
}

void hiros::track::OrientationFilter::computeOrientation(hiros::skeletons::types::MIMU& t_mimu,
                                                         const double& t_time,
                                                         const double& t_cutoff)
{
  // init
  if (std::isnan(m_last_time)) {
    m_last_time = t_time;
    m_last_orientation = t_mimu.orientation;
    m_last_velocity = std::isnan(t_mimu.angular_velocity.x()) ? tf2::Vector3(0, 0, 0) : t_mimu.angular_velocity;
    return;
  }

  // filter
  double weight = std::max(0., std::min((t_time - m_last_time) * t_cutoff, 1.));
  m_last_orientation = m_last_orientation.slerp(t_mimu.orientation, weight);
  t_mimu.orientation = m_last_orientation;
  m_last_time = t_time;
}

void hiros::track::OrientationFilter::computeVelocity(hiros::skeletons::types::MIMU& t_mimu,
                                                      const tf2::Quaternion& t_prev_orientation,
                                                      const double& t_prev_time,
                                                      const double& t_cutoff)
{
  // init
  if (std::isnan(t_prev_time)) {
    t_mimu.angular_velocity = tf2::Vector3(0, 0, 0);
    return;
  }

  // compute angular velocities
  double delta_roll, delta_pitch, delta_yaw;
  tf2::Vector3 curr_vel;
  double dt = m_last_time - t_prev_time;

  tf2::Matrix3x3 m(m_last_orientation * t_prev_orientation.inverse());
  m.getEulerYPR(delta_yaw, delta_pitch, delta_roll);

  curr_vel.setX(delta_roll / dt);
  curr_vel.setY(delta_pitch / dt);
  curr_vel.setZ(delta_yaw / dt);

  // filter
  double weight = std::max(0., std::min(dt * t_cutoff, 1.));

  m_last_velocity = m_last_velocity.lerp(curr_vel, weight);
  t_mimu.angular_velocity = m_last_velocity;
}

hiros::track::Filter::Filter(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff)
{
  init(t_skeleton, t_cutoff);
}

void hiros::track::Filter::init(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff)
{
  if (!m_initialized) {
    for (auto& mkg : t_skeleton.marker_groups) {
      for (auto& mk : mkg.markers) {
        m_marker_filters[mkg.id][mk.id] = MarkerFilter();
        m_marker_filters[mkg.id][mk.id].filter(mk.point, t_skeleton.src_time, t_cutoff);
      }
    }

    for (auto& org : t_skeleton.orientation_groups) {
      for (auto& o : org.orientations) {
        m_orientation_filters[org.id][o.id] = OrientationFilter();
        m_orientation_filters[org.id][o.id].filter(o.mimu, t_skeleton.src_time, t_cutoff);
      }
    }

    m_cutoff = t_cutoff;
    m_initialized = true;
  }
}

void hiros::track::Filter::updateMarkerFilters(hiros::skeletons::types::Skeleton& t_skeleton)
{
  for (const auto& mkg : m_marker_filters) {
    auto mkg_id = mkg.first;

    if (t_skeleton.hasMarkerGroup(mkg_id)) {
      auto& mkg = t_skeleton.getMarkerGroup(mkg_id);

      for (const auto& mk : m_marker_filters.at(mkg_id)) {
        auto mk_id = mk.first;

        if (!mkg.hasMarker(mk_id)) {
          // erase empty markers
          m_marker_filters.at(mkg_id).erase(mk_id);
        }
      }
    }
    else {
      // erase empty marker groups
      m_marker_filters.erase(mkg_id);
    }
  }

  for (auto& mkg : t_skeleton.marker_groups) {
    if (m_marker_filters.count(mkg.id) == 0) {
      // new marker group
      m_marker_filters[mkg.id] = {};
    }

    for (auto& mk : mkg.markers) {
      if (m_marker_filters[mkg.id].count(mk.id) == 0) {
        // new marker
        m_marker_filters[mkg.id][mk.id] = MarkerFilter();
      }
      m_marker_filters[mkg.id][mk.id].filter(mk.point, t_skeleton.src_time, m_cutoff);
    }
  }
}

void hiros::track::Filter::updateOrientationFilters(hiros::skeletons::types::Skeleton& t_skeleton)
{
  for (const auto& org : m_orientation_filters) {
    auto org_id = org.first;

    if (t_skeleton.hasOrientationGroup(org_id)) {
      auto& org = t_skeleton.getOrientationGroup(org_id);

      for (const auto& o : m_orientation_filters.at(org_id)) {
        auto or_id = o.first;

        if (!org.hasOrientation(or_id)) {
          // erase empty orientations
          m_orientation_filters.at(org_id).erase(or_id);
        }
      }
    }
    else {
      // erase empty orientation groups
      m_orientation_filters.erase(org_id);
    }
  }

  for (auto& org : t_skeleton.orientation_groups) {
    if (m_orientation_filters.count(org.id) == 0) {
      // new orientation group
      m_orientation_filters[org.id] = {};
    }

    for (auto& o : org.orientations) {
      if (m_orientation_filters[org.id].count(o.id) == 0) {
        // new orientation
        m_orientation_filters[org.id][o.id] = OrientationFilter();
      }
      m_orientation_filters[org.id][o.id].filter(o.mimu, t_skeleton.src_time, m_cutoff);
    }
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
  updateOrientationFilters(t_skeleton);
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

  // reset marker positions to the values before filtering
  for (auto& mkg : t_skeleton.marker_groups) {
    for (auto& mk : mkg.markers) {
      mk.point.position = original_skel.getMarkerGroup(mkg.id).getMarker(mk.id).point.position;
    }
  }

  // reset MIMU orientations to the values before filtering
  for (auto& org : t_skeleton.orientation_groups) {
    for (auto& o : org.orientations) {
      o.mimu.orientation = original_skel.getOrientationGroup(org.id).getOrientation(o.id).mimu.orientation;
    }
  }
}

void hiros::track::Filter::updVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff) const
{
  auto tmp = *this;
  tmp.updVelAndAcc(t_skeleton, t_cutoff);
}
