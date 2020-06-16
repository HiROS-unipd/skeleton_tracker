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
      void minWithIndex(const cv::Mat_<double>& t_mat, double& t_min, unsigned int& t_row, unsigned int& t_col);

      bool isEmpty(const hiros::skeletons::types::Skeleton& t_skeleton);
      int numberOfKeypoints(const hiros::skeletons::types::Skeleton& t_skeleton);
      void initializeVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton);
      bool hasKeypoint(const hiros::skeletons::types::Skeleton& t_skeleton,
                       const int& t_keypoint_group_id,
                       const int& t_keypoint_id);
      hiros::skeletons::types::Keypoint findKeypoint(const hiros::skeletons::types::Skeleton& t_skeleton,
                                                     const int& t_keypoint_group_id,
                                                     const int& t_keypoint_id);
      void merge(hiros::skeletons::types::Skeleton& t_s1, const hiros::skeletons::types::Skeleton& t_s2);
      void merge(hiros::skeletons::types::Skeleton& t_sk,
                 const hiros::skeletons::types::KeypointGroup& t_kpg,
                 const hiros::skeletons::types::Keypoint& t_kp);
      double magnitude(const hiros::skeletons::types::Vector& t_v);
      double distance(const hiros::skeletons::types::Vector& t_v1, const hiros::skeletons::types::Vector& t_v2);

    } // namespace utils
  } // namespace track
} // namespace hiros

#endif
