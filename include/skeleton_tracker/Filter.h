#ifndef hiros_skeleton_tracker_Filter_h
#define hiros_skeleton_tracker_Filter_h

// Standard dependencies
#include <map>

// Custom external dependencies
#include "skeletons/types.h"

// Custom Internal dependencies
#include "skeleton_tracker/utils.h"

namespace hiros {
  namespace track {

    class KinematicStateFilter
    {
    public:
      void filter(hiros::skeletons::types::KinematicState& t_state, const double& t_time, const double& t_cutoff);

    private:
      void computePose(hiros::skeletons::types::KinematicState& t_state, const double& t_time, const double& t_cutoff);

      void computeVelocity(hiros::skeletons::types::KinematicState& t_state,
                           const hiros::skeletons::types::KinematicState& t_prev_state,
                           const double& t_prev_time,
                           const double& t_cutoff);

      void computeAcceleration(hiros::skeletons::types::KinematicState& t_state,
                               const hiros::skeletons::types::KinematicState& t_prev_state,
                               const double& t_prev_time,
                               const double& t_cutoff);

      double m_last_time{std::numeric_limits<double>::quiet_NaN()};
      hiros::skeletons::types::KinematicState m_last_state{};
    };

    class Filter
    {
    public:
      Filter() {}
      Filter(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff);
      ~Filter() {}

      // Get filtered value and update the filter's internal state
      void filter(hiros::skeletons::types::Skeleton& t_skeleton,
                  const double& t_cutoff = std::numeric_limits<double>::quiet_NaN());

      // Get filtered value without modifying the filter's internal state
      void filter(hiros::skeletons::types::Skeleton& t_skeleton,
                  const double& t_cutoff = std::numeric_limits<double>::quiet_NaN()) const;

      // Get filtered velocity and acceleration values and update the filter's internal state
      void updVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton,
                        const double& t_cutoff = std::numeric_limits<double>::quiet_NaN());

      // Get filtered velocity and acceleration values without modifying the filter's internal state
      void updVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton,
                        const double& t_cutoff = std::numeric_limits<double>::quiet_NaN()) const;

    private:
      void init(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_cutoff);

      void updateMarkerFilters(hiros::skeletons::types::Skeleton& t_skeleton);
      void updateLinkFilters(hiros::skeletons::types::Skeleton& t_skeleton);

      // <marker_id, marker_filter>
      std::map<int, KinematicStateFilter> m_marker_filters{};
      // <link_id, link_filter>
      std::map<int, KinematicStateFilter> m_link_filters{};

      double m_cutoff{};
      bool m_initialized{false};
    };

  } // namespace track
} // namespace hiros

#endif
