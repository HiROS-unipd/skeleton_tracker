// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/Filter.h"

void hiros::track::StateSpaceFilter3::filter(hiros::skeletons::types::Point& t_point,
                                             const double& t_time,
                                             const double& t_cutoff)
{
  t_point.position.x = filters[0].filter(t_point.position.x, t_time, t_cutoff);
  t_point.position.y = filters[1].filter(t_point.position.y, t_time, t_cutoff);
  t_point.position.z = filters[2].filter(t_point.position.z, t_time, t_cutoff);

  t_point.velocity.x = filters[0].getFilteredFirstDerivative();
  t_point.velocity.y = filters[1].getFilteredFirstDerivative();
  t_point.velocity.z = filters[2].getFilteredFirstDerivative();

  t_point.acceleration.x = filters[0].getFilteredSecondDerivative();
  t_point.acceleration.y = filters[1].getFilteredSecondDerivative();
  t_point.acceleration.z = filters[2].getFilteredSecondDerivative();
}

hiros::track::Filter::Filter(hiros::skeletons::types::Skeleton& t_skeleton,
                             const double& t_time,
                             const double& t_cutoff)
  : m_cutoff(t_cutoff)
{
  for (auto& skg : t_skeleton.skeleton_parts) {
    for (auto& kp : skg.second.keypoints) {
      m_filters[skg.first][kp.first] = StateSpaceFilter3();
      m_filters[skg.first][kp.first].filter(kp.second.point, t_time, m_cutoff);
    }
  }
}

hiros::track::Filter::~Filter() {}

void hiros::track::Filter::filter(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_time)
{
  for (auto skg_id = m_filters.begin()->first; skg_id < m_filters.end()->first; ++skg_id) {
    if (m_filters.find(skg_id) != m_filters.end()) {
      for (auto kp_id = m_filters.at(skg_id).begin()->first; kp_id < m_filters.at(skg_id).end()->first; ++kp_id) {
        // erase empty keypoints
        if (m_filters.at(skg_id).find(kp_id) != m_filters.at(skg_id).end()
            && !hiros::skeletons::utils::hasKeypoint(t_skeleton, skg_id, kp_id)) {
          m_filters.at(skg_id).erase(kp_id);
        }
      }

      // erase empty skeleton groups
      if (m_filters.at(skg_id).empty()) {
        m_filters.erase(skg_id);
      }
    }
  }

  for (auto& skg : t_skeleton.skeleton_parts) {
    // new skeleton group
    if (m_filters.find(skg.first) == m_filters.end()) {
      m_filters[skg.first] = {};
      for (auto& kp : skg.second.keypoints) {
        m_filters[skg.first][kp.first] = StateSpaceFilter3();
        m_filters[skg.first][kp.first].filter(kp.second.point, t_time, m_cutoff);
      }
    }
    else {
      for (auto& kp : skg.second.keypoints) {
        // new keypoint
        if (m_filters[skg.first].find(kp.first) == m_filters[skg.first].end()) {
          m_filters[skg.first][kp.first] = StateSpaceFilter3();
        }

        m_filters[skg.first][kp.first].filter(kp.second.point, t_time, m_cutoff);
      }
    }
  }
}
