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
      hiros::skeletons::types::Vector wavg(const hiros::skeletons::types::Vector& t_v1,
                                           const hiros::skeletons::types::Vector& t_v2,
                                           const double& t_w1,
                                           const double& t_w2);
      hiros::skeletons::types::Marker wavg(const hiros::skeletons::types::Marker& t_mk1,
                                           const hiros::skeletons::types::Marker& t_mk2,
                                           const double& t_w1 = 1,
                                           const double& t_w2 = 1);

    } // namespace utils
  } // namespace track
} // namespace hiros

#endif
