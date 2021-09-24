#ifndef hiros_skeleton_tracker_Filter_h
#define hiros_skeleton_tracker_Filter_h

// Standard dependencies
#include <map>

// Custom external dependencies
#include "rtb/Filter/StateSpaceFilter.h"
#include "skeletons/types.h"

// Custom Internal dependencies
#include "skeleton_tracker/utils.h"

namespace hiros {
  namespace track {

    struct StateSpaceFilter3
    {
      void filter(hiros::skeletons::types::Point& t_point, const double& t_time, const double& t_cutoff);

      std::array<rtb::Filter::StateSpaceFilter<double>, 3> filters;
    };

    class OrientationFilter
    {
    public:
      void filter(hiros::skeletons::types::MIMU& t_mimu, const double& t_time, const double& t_cutoff);

    private:
      void computeOrientation(hiros::skeletons::types::MIMU& t_mimu, const double& t_time, const double& t_cutoff);
      void computeVelocity(hiros::skeletons::types::MIMU& t_mimu,
                           const tf2::Quaternion& t_prev_orientation,
                           const double& t_prev_time);

      double m_last_time{std::numeric_limits<double>::quiet_NaN()};
      tf2::Quaternion m_last_orientation{};
      tf2::Vector3 m_last_velocity{};
    };

    class Filter
    {
    public:
      Filter() {}
      Filter(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_time, const double& t_cutoff);
      Filter(utils::StampedSkeleton& t_skeleton, const double& t_cutoff);
      ~Filter() {}

      void updateMarkerFilters(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_time);
      void updateOrientationFilters(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_time);

      // Get filtered value and update the filter's internal state
      void filter(hiros::skeletons::types::Skeleton& t_skeleton,
                  const double& t_time,
                  const double& t_cutoff = std::numeric_limits<double>::quiet_NaN());
      void filter(utils::StampedSkeleton& t_skeleton,
                  const double& t_cutoff = std::numeric_limits<double>::quiet_NaN());

      // Get filtered value without modifying the filter's internal state
      void filter(hiros::skeletons::types::Skeleton& t_skeleton,
                  const double& t_time,
                  const double& t_cutoff = std::numeric_limits<double>::quiet_NaN()) const;
      void filter(utils::StampedSkeleton& t_skeleton,
                  const double& t_cutoff = std::numeric_limits<double>::quiet_NaN()) const;

      // Get filtered velocity and acceleration values and update the filter's internal state
      void updVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton,
                        const double& t_time,
                        const double& t_cutoff = std::numeric_limits<double>::quiet_NaN());
      void updVelAndAcc(utils::StampedSkeleton& t_skeleton,
                        const double& t_cutoff = std::numeric_limits<double>::quiet_NaN());

      // Get filtered velocity and acceleration values without modifying the filter's internal state
      void updVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton,
                        const double& t_time,
                        const double& t_cutoff = std::numeric_limits<double>::quiet_NaN()) const;
      void updVelAndAcc(utils::StampedSkeleton& t_skeleton,
                        const double& t_cutoff = std::numeric_limits<double>::quiet_NaN()) const;

    private:
      void init(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_time, const double& t_cutoff);

      // <marker_group_id, <marker_id, marker_filter>>
      std::map<int, std::map<int, StateSpaceFilter3>> m_marker_filters{};
      // <orientation_group_id, <orientation_id, orientation_filter>>
      std::map<int, std::map<int, OrientationFilter>> m_orientation_filters{};

      double m_cutoff{};
      bool m_initialized{false};
    };

  } // namespace track
} // namespace hiros

#endif
