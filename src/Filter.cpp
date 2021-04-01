// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/Filter.h"

void hiros::track::StateSpaceFilter3::filter(hiros::skeletons::types::Point& t_point,
                                             const double& t_time,
                                             const double& t_cutoff)
{
  t_point.position.setX(filters[0].filter(t_point.position.x(), t_time, t_cutoff));
  t_point.position.setY(filters[1].filter(t_point.position.y(), t_time, t_cutoff));
  t_point.position.setZ(filters[2].filter(t_point.position.z(), t_time, t_cutoff));

  t_point.velocity.setX(filters[0].getFilteredFirstDerivative());
  t_point.velocity.setY(filters[1].getFilteredFirstDerivative());
  t_point.velocity.setZ(filters[2].getFilteredFirstDerivative());

  t_point.acceleration.setX(filters[0].getFilteredSecondDerivative());
  t_point.acceleration.setY(filters[1].getFilteredSecondDerivative());
  t_point.acceleration.setZ(filters[2].getFilteredSecondDerivative());
}

hiros::track::Filter::Filter(hiros::skeletons::types::MarkerSkeleton& t_skeleton,
                             const double& t_time,
                             const double& t_cutoff)
  : m_cutoff(t_cutoff)
{
  for (auto& mkg : t_skeleton.marker_groups) {
    for (auto& mk : mkg.second.markers) {
      m_filters[mkg.first][mk.first] = StateSpaceFilter3();
      m_filters[mkg.first][mk.first].filter(mk.second.point, t_time, m_cutoff);
    }
  }
}

hiros::track::Filter::~Filter() {}

void hiros::track::Filter::filter(hiros::skeletons::types::MarkerSkeleton& t_skeleton, const double& t_time)
{
  for (const auto& mkg : m_filters) {
    auto mkg_id = mkg.first;

    if (m_filters.count(mkg_id) > 0) {
      for (const auto& mk : m_filters.at(mkg_id)) {
        auto mk_id = mk.first;

        // erase empty markers
        if (m_filters.at(mkg_id).count(mk_id) > 0 && !hiros::skeletons::utils::hasMarker(t_skeleton, mkg_id, mk_id)) {
          m_filters.at(mkg_id).erase(mk_id);
        }
      }

      // erase empty marker groups
      if (m_filters.at(mkg_id).empty()) {
        m_filters.erase(mkg_id);
      }
    }
  }

  for (auto& mkg : t_skeleton.marker_groups) {
    auto mkg_id = mkg.first;

    // new marker group
    if (m_filters.count(mkg_id) == 0) {
      m_filters[mkg_id] = {};
      for (auto& mk : mkg.second.markers) {
        m_filters[mkg.first][mk.first] = StateSpaceFilter3();
        m_filters[mkg.first][mk.first].filter(mk.second.point, t_time, m_cutoff);
      }
    }
    else {
      for (auto& mk : mkg.second.markers) {
        auto mk_id = mk.first;

        // new marker
        if (m_filters[mkg_id].count(mk_id) == 0) {
          m_filters[mkg_id][mk_id] = StateSpaceFilter3();
        }

        m_filters[mkg_id][mk_id].filter(mk.second.point, t_time, m_cutoff);
      }
    }
  }
}
