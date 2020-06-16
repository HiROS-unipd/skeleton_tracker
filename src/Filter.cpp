// Internal dependencies
#include "skeleton_tracker/Filter.h"
#include "skeleton_tracker/utils.h"

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
    for (auto& kp : skg.keypoints) {
      m_filters[skg.id][kp.id] = StateSpaceFilter3();
      m_filters[skg.id][kp.id].filter(kp.point, t_time, m_cutoff);
    }
  }
}

hiros::track::Filter::~Filter() {}

void hiros::track::Filter::filter(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_time)
{
  for (auto& skg : t_skeleton.skeleton_parts) {
    // new skeleton group
    if (m_filters.find(skg.id) == m_filters.end()) {
      for (auto& kp : skg.keypoints) {
        m_filters[skg.id][kp.id] = StateSpaceFilter3();
        m_filters[skg.id][kp.id].filter(kp.point, t_time, m_cutoff);
      }
    }
    else {
      for (auto& kp : skg.keypoints) {
        // new keypoint
        if (m_filters[skg.id].find(kp.id) == m_filters[skg.id].end()) {
          m_filters[skg.id][kp.id] = StateSpaceFilter3();
        }

        m_filters[skg.id][kp.id].filter(kp.point, t_time, m_cutoff);
      }
    }
  }
}
