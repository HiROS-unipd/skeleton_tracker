#ifndef hiros_skeleton_tracker_utils_h
#define hiros_skeleton_tracker_utils_h

// OpenCV dependencies
#include "opencv2/opencv.hpp"

// Custom External Packages dependencies
#include "hiros_skeleton_msgs/Skeleton.h"
#include "skeletons/types.h"

namespace hiros {
  namespace track {
    namespace utils {

      struct StampedSkeleton
      {
        StampedSkeleton(const hiros::skeletons::types::Skeleton t_skeleton = {},
                        const ros::Time& t_src_time = {},
                        const std::string& t_src_frame = {})
          : skeleton(t_skeleton)
          , src_time(t_src_time)
          , src_frame(t_src_frame)
        {}

        hiros::skeletons::types::Skeleton skeleton;
        ros::Time src_time;
        std::string src_frame;
      };

      bool isNaN(const cv::Mat_<double>& t_mat);
      double min(const cv::Mat_<double>& t_mat);
      double max(const cv::Mat_<double>& t_mat);
      void minWithIndex(const cv::Mat_<double>& t_mat, double& t_min, unsigned int& t_row, unsigned int& t_col);

      bool matchMunkres(const cv::Mat_<int>& t_munkres_matrix, const unsigned int& t_row, const unsigned int& t_col);

      int getSkeletonIndexFromId(const std::vector<StampedSkeleton>& t_vec, const int& t_id);
      std::shared_ptr<StampedSkeleton> getSkeletonFromId(const std::vector<StampedSkeleton>& t_vec, const int& t_id);

      bool isEmpty(const hiros::skeletons::types::Skeleton& t_skeleton);
      bool isEmpty(const hiros_skeleton_msgs::Skeleton& t_skeleton);

      void merge(StampedSkeleton& t_s1,
                 const StampedSkeleton& t_s2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      void merge(hiros::skeletons::types::Skeleton& t_s1,
                 const hiros::skeletons::types::Skeleton& t_s2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      void merge(hiros::skeletons::types::Skeleton& t_sk1,
                 const hiros::skeletons::types::MarkerGroup& t_mkg2,
                 const hiros::skeletons::types::Marker& t_mk2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);
      void merge(hiros::skeletons::types::Skeleton& t_sk1,
                 const hiros::skeletons::types::OrientationGroup& t_org2,
                 const hiros::skeletons::types::Orientation& t_or2,
                 const double& t_w1 = 1,
                 const double& t_w2 = 1,
                 const bool& t_weight_by_confidence = false);

      tf2::Vector3 wavg(const tf2::Vector3& t_v1, const tf2::Vector3& t_v2, const double& t_w1, const double& t_w2);
      hiros::skeletons::types::Marker wavg(const hiros::skeletons::types::Marker& t_mk1,
                                           const hiros::skeletons::types::Marker& t_mk2,
                                           const double& t_w1 = 1,
                                           const double& t_w2 = 1);
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
