#ifndef hiros_skeleton_tracker_Filter_h
#define hiros_skeleton_tracker_Filter_h

// Standard dependencies
#include <map>

// Custom external dependencies
#include "rtb/Filter/StateSpaceFilter.h"
#include "skeletons/types.h"

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
      Filter(hiros::skeletons::types::MarkerSkeleton& t_skeleton, const double& t_time, const double& t_cutoff);
      ~Filter();

      void filter(hiros::skeletons::types::MarkerSkeleton& t_skeleton, const double& t_time);

    private:
      // <marker_group_id, <marker_id, filter>>
      std::map<int, std::map<int, StateSpaceFilter3>> m_filters;
      double m_cutoff;
    };

  } // namespace track
} // namespace hiros

#endif
