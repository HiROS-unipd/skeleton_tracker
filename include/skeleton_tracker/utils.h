#ifndef hiros_skeleton_tracker_utils_h
#define hiros_skeleton_tracker_utils_h

// OpenCV dependencies
#include "opencv2/opencv.hpp"

// Custom External Packages dependencies
#include "skeletons/types.h"

namespace hiros {
  namespace track {
    namespace utils {

      bool isNaN(const cv::Mat_<double>& t_mat);
      double max(const cv::Mat_<double>& t_mat);
      void replaceNansWithMax(cv::Mat_<double>& t_mat);
      void replaceNans(cv::Mat_<double>& t_mat);

      bool isEmpty(const hiros::skeletons::types::Skeleton& t_skeleton);
      int numberOfKeypoints(const hiros::skeletons::types::Skeleton& t_skeleton);
      void initializeVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton);
      hiros::skeletons::types::Keypoint findKeypoint(const hiros::skeletons::types::Skeleton& t_skeleton,
                                                     const int& t_keypoint_group_id,
                                                     const int& t_keypoint_id);
      double distance(const hiros::skeletons::types::Vector& t_v1, const hiros::skeletons::types::Vector& t_v2);

    } // namespace utils
  } // namespace track
} // namespace hiros

#endif
