#ifndef hiros_skeleton_tracker_utils_h
#define hiros_skeleton_tracker_utils_h

// OpenCV dependencies
#include "opencv2/opencv.hpp"

// Custom External Packages dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"
#include "skeletons/types.h"

namespace hiros {
  namespace track {
    namespace utils {

      bool isNaN(const cv::Mat_<double>& t_mat);
      double min(const cv::Mat_<double>& t_mat);
      double max(const cv::Mat_<double>& t_mat);
      void minWithIndex(const cv::Mat_<double>& t_mat, double& t_min, unsigned int& t_row, unsigned int& t_col);

      bool matchMunkres(const cv::Mat_<int>& t_munkres_matrix, const unsigned int& t_row, const unsigned int& t_col);

      bool isEmpty(const hiros::skeletons::types::Skeleton& t_skeleton);
      bool isEmpty(const hiros_skeleton_msgs::Skeleton& t_skeleton);

      bool isEmpty(const hiros::skeletons::types::SkeletonGroup& t_skeleton_group);
      bool isEmpty(const hiros_skeleton_msgs::SkeletonGroup& t_skeleton_group);

      hiros::skeletons::types::Skeleton predict(const hiros::skeletons::types::Skeleton& t_skeleton,
                                                const double& t_current_time);
      void predict(hiros::skeletons::types::KinematicState& t_state, const double& t_dt);

      void merge(hiros::skeletons::types::Skeleton& t_sk1,
                 const hiros::skeletons::types::Skeleton& t_sk2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      void merge(hiros::skeletons::types::Skeleton& t_sk1,
                 const hiros::skeletons::types::Marker& t_mk2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      void merge(hiros::skeletons::types::Skeleton& t_sk1,
                 const hiros::skeletons::types::Link& t_lk2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);

      double wavg(const double& t_e1, const double& t_e2, const double& t_w1 = 1, const double& t_w2 = 1);
      tf2::Vector3 wavg(const tf2::Vector3& t_v1, const tf2::Vector3& t_v2, const double& t_w1, const double& t_w2);
      tf2::Quaternion
      wavg(const tf2::Quaternion& t_q1, const tf2::Quaternion& t_q2, const double& t_w1 = 1, const double& t_w2 = 1);

      hiros::skeletons::types::KinematicState wavg(const hiros::skeletons::types::KinematicState& t_ks1,
                                                   const hiros::skeletons::types::KinematicState& t_ks2,
                                                   const double& t_w1 = 1,
                                                   const double& t_w2 = 1);

      hiros::skeletons::types::Marker wavg(const hiros::skeletons::types::Marker& t_mk1,
                                           const hiros::skeletons::types::Marker& t_mk2,
                                           const double& t_w1 = 1,
                                           const double& t_w2 = 1);

      hiros::skeletons::types::Link wavg(const hiros::skeletons::types::Link& t_lk1,
                                         const hiros::skeletons::types::Link& t_lk2,
                                         const double& t_w1 = 1,
                                         const double& t_w2 = 1);

      ros::Time avgSrcTime(const hiros::skeletons::types::SkeletonGroup& t_skel_group);
      ros::Time avgSrcTime(const hiros_skeleton_msgs::SkeletonGroup& t_skel_group);
      ros::Time oldestSrcTime(const hiros::skeletons::types::SkeletonGroup& t_skel_group);
      ros::Time oldestSrcTime(const hiros_skeleton_msgs::SkeletonGroup& t_skel_group);
      ros::Time newestSrcTime(const hiros::skeletons::types::SkeletonGroup& t_skel_group);
      ros::Time newestSrcTime(const hiros_skeleton_msgs::SkeletonGroup& t_skel_group);

    } // namespace utils
  } // namespace track
} // namespace hiros

#endif
