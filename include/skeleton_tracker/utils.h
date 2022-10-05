#ifndef hiros_skeleton_tracker_utils_h
#define hiros_skeleton_tracker_utils_h

// ROS dependencies
#include <rclcpp/time.hpp>

// OpenCV dependencies
#include <opencv2/opencv.hpp>

// Custom External Packages dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"
#include "skeletons/types.h"

namespace hiros {
namespace skeletons {
namespace utils {

bool isNaN(const cv::Mat_<double>& mat);
double min(const cv::Mat_<double>& mat);
double max(const cv::Mat_<double>& mat);
void minWithIndex(const cv::Mat_<double>& mat, double& min, unsigned int& row,
                  unsigned int& col);

bool matchMunkres(const cv::Mat_<int>& munkres_matrix, const unsigned int& row,
                  const unsigned int& col);

bool isEmpty(const hiros::skeletons::types::Skeleton& skeleton);
bool isEmpty(const hiros_skeleton_msgs::msg::Skeleton& skeleton);

bool isEmpty(const hiros::skeletons::types::SkeletonGroup& skeleton_group);
bool isEmpty(const hiros_skeleton_msgs::msg::SkeletonGroup& skeleton_group);

hiros::skeletons::types::Skeleton predict(
    const hiros::skeletons::types::Skeleton& skeleton,
    const double& current_time);
void predict(hiros::skeletons::types::KinematicState& state, const double& dt);

void merge(hiros::skeletons::types::Skeleton& sk1,
           const hiros::skeletons::types::Skeleton& sk2, const double& w1 = 1,
           const double& w2 = 1, const bool& weight_by_confidence = false);
void merge(hiros::skeletons::types::Skeleton& sk1,
           const hiros::skeletons::types::Marker& mk2, const double& w1 = 1,
           const double& w2 = 1, const bool& weight_by_confidence = false);
void merge(hiros::skeletons::types::Skeleton& sk1,
           const hiros::skeletons::types::Link& lk2, const double& w1 = 1,
           const double& w2 = 1, const bool& weight_by_confidence = false);

void alignLinkOrientation(hiros::skeletons::types::Skeleton& sk,
                          const int& lk_id);
tf2::Vector3 closestCartesianAxis(const tf2::Vector3& vec);

double wavg(const double& e1, const double& e2, const double& w1 = 1,
            const double& w2 = 1);
tf2::Vector3 wavg(const tf2::Vector3& v1, const tf2::Vector3& v2,
                  const double& w1 = 1, const double& w2 = 1);
tf2::Quaternion wavg(const tf2::Quaternion& q1, const tf2::Quaternion& q2,
                     const double& w1 = 1, const double& w2 = 1);

hiros::skeletons::types::KinematicState wavg(
    const hiros::skeletons::types::KinematicState& ks1,
    const hiros::skeletons::types::KinematicState& ks2, const double& w1 = 1,
    const double& w2 = 1);

hiros::skeletons::types::Marker wavg(const hiros::skeletons::types::Marker& mk1,
                                     const hiros::skeletons::types::Marker& mk2,
                                     const double& w1 = 1,
                                     const double& w2 = 1);

hiros::skeletons::types::Link wavg(const hiros::skeletons::types::Link& lk1,
                                   const hiros::skeletons::types::Link& lk2,
                                   const double& w1 = 1, const double& w2 = 1);

rclcpp::Time avgSrcTime(
    const hiros::skeletons::types::SkeletonGroup& skel_group);
rclcpp::Time avgSrcTime(
    const hiros_skeleton_msgs::msg::SkeletonGroup& skel_group);
rclcpp::Time oldestSrcTime(
    const hiros::skeletons::types::SkeletonGroup& skel_group);
rclcpp::Time oldestSrcTime(
    const hiros_skeleton_msgs::msg::SkeletonGroup& skel_group);
rclcpp::Time newestSrcTime(
    const hiros::skeletons::types::SkeletonGroup& skel_group);
rclcpp::Time newestSrcTime(
    const hiros_skeleton_msgs::msg::SkeletonGroup& skel_group);

}  // namespace utils
}  // namespace skeletons
}  // namespace hiros

#endif
