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

      bool isEmpty(const hiros::skeletons::types::MarkerSkeleton& t_skeleton);
      int numberOfMarkers(const hiros::skeletons::types::MarkerSkeleton& t_skeleton);
      void merge(hiros::skeletons::types::MarkerSkeleton& t_s1,
                 const hiros::skeletons::types::MarkerSkeleton& t_s2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      void merge(hiros::skeletons::types::MarkerSkeleton& t_sk,
                 const hiros::skeletons::types::MarkerGroup& t_mkg,
                 const hiros::skeletons::types::Marker& t_mk,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      tf2::Vector3 wavg(const tf2::Vector3& t_v1, const tf2::Vector3& t_v2, const double& t_w1, const double& t_w2);
      hiros::skeletons::types::Marker wavg(const hiros::skeletons::types::Marker& t_mk1,
                                           const hiros::skeletons::types::Marker& t_mk2,
                                           const double& t_w1 = 1,
                                           const double& t_w2 = 1);

      bool isEmpty(const hiros::skeletons::types::OrientationSkeleton& t_skeleton);
      int numberOfOrientations(const hiros::skeletons::types::OrientationSkeleton& t_skeleton);
      void merge(hiros::skeletons::types::OrientationSkeleton& t_s1,
                 const hiros::skeletons::types::OrientationSkeleton& t_s2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      void merge(hiros::skeletons::types::OrientationSkeleton& t_sk,
                 const hiros::skeletons::types::OrientationGroup& t_org,
                 const hiros::skeletons::types::Orientation& t_or,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      tf2::Quaternion
      wavg(const tf2::Quaternion& t_q1, const tf2::Quaternion& t_q2, const double& t_w1 = 1, const double& t_w2 = 1);
      hiros::skeletons::types::Orientation wavg(const hiros::skeletons::types::Orientation& t_or1,
                                                const hiros::skeletons::types::Orientation& t_or2,
                                                const double& t_w1 = 1,
                                                const double& t_w2 = 1);

    } // namespace utils
  } // namespace track
} // namespace hiros

#endif
