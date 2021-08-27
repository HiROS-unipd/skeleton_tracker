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

hiros::track::Filter::Filter(hiros::skeletons::types::Skeleton& t_skeleton,
                             const double& t_time,
                             const double& t_cutoff)
{
  init(t_skeleton, t_time, t_cutoff);
}

void hiros::track::Filter::init(hiros::skeletons::types::Skeleton& t_skeleton,
                                const double& t_time,
                                const double& t_cutoff)
{
  if (!m_initialized) {
    for (auto& mkg : t_skeleton.marker_groups) {
      for (auto& mk : mkg.markers) {
        m_filters[mkg.id][mk.id] = StateSpaceFilter3();
        m_filters[mkg.id][mk.id].filter(mk.point, t_time, t_cutoff);
      }
    }
    m_cutoff = t_cutoff;
    m_initialized = true;
  }
}

hiros::track::Filter::Filter(utils::StampedSkeleton& t_skeleton, const double& t_cutoff)
{
  Filter(t_skeleton.skeleton, t_skeleton.src_time.toSec(), t_cutoff);
}

void hiros::track::Filter::filter(hiros::skeletons::types::Skeleton& t_skeleton,
                                  const double& t_time,
                                  const double& t_cutoff)
{
  if (!m_initialized) {
    if (std::isnan(t_cutoff)) {
      std::cerr << "hiros::track::Filter Warning: cutoff = NaN" << std::endl;
      return;
    }
    init(t_skeleton, t_time, t_cutoff);
  }

  for (const auto& mkg : m_filters) {
    auto mkg_id = mkg.first;

    if (t_skeleton.hasMarkerGroup(mkg_id)) {
      auto& mkg = t_skeleton.getMarkerGroup(mkg_id);

      for (const auto& mk : m_filters.at(mkg_id)) {
        auto mk_id = mk.first;

        if (!mkg.hasMarker(mk_id)) {
          // erase empty markers
          m_filters.at(mkg_id).erase(mk_id);
        }
      }
    }
    else {
      // erase empty marker groups
      m_filters.erase(mkg_id);
    }
  }

  for (auto& mkg : t_skeleton.marker_groups) {
    if (m_filters.count(mkg.id) == 0) {
      // new marker group
      m_filters[mkg.id] = {};
    }

    for (auto& mk : mkg.markers) {
      if (m_filters[mkg.id].count(mk.id) == 0) {
        // new marker
        m_filters[mkg.id][mk.id] = StateSpaceFilter3();
      }
      m_filters[mkg.id][mk.id].filter(mk.point, t_time, m_cutoff);
    }
  }
}

void hiros::track::Filter::filter(utils::StampedSkeleton& t_skeleton, const double& t_cutoff)
{
  filter(t_skeleton.skeleton, t_skeleton.src_time.toSec(), t_cutoff);
}
