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

    class Filter
    {
    public:
      Filter() {}
      Filter(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_time, const double& t_cutoff);
      Filter(utils::StampedSkeleton& t_skeleton, const double& t_cutoff);
      ~Filter() {}

      void filter(hiros::skeletons::types::Skeleton& t_skeleton,
                  const double& t_time,
                  const double& t_cutoff = std::numeric_limits<double>::quiet_NaN());
      void filter(utils::StampedSkeleton& t_skeleton,
                  const double& t_cutoff = std::numeric_limits<double>::quiet_NaN());

    private:
      void init(hiros::skeletons::types::Skeleton& t_skeleton, const double& t_time, const double& t_cutoff);

      // <marker_group_id, <marker_id, filter>>
      std::map<int, std::map<int, StateSpaceFilter3>> m_filters{};
      double m_cutoff{};
      bool m_initialized{false};
    };

  } // namespace track
} // namespace hiros

#endif
