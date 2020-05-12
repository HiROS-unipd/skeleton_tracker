// Internal dependencies
#include "skeleton_tracker/utils.h"

bool hiros::track::utils::isNaN(const cv::Mat_<double>& t_mat)
{
  for (int r = 0; r < t_mat.rows; ++r) {
    for (int c = 0; c < t_mat.cols; ++c) {
      if (!std::isnan(t_mat(r, c))) {
        return false;
      }
    }
  }

  return true;
}

double hiros::track::utils::max(const cv::Mat_<double>& t_mat)
{
  double max = -std::numeric_limits<double>::max();

  for (int r = 0; r < t_mat.rows; ++r) {
    for (int c = 0; c < t_mat.cols; ++c) {
      if (!std::isnan(t_mat(r, c))) {
        max = std::max(t_mat(r, c), max);
      }
    }
  }

  return max;
}

void hiros::track::utils::replaceNansWithMax(cv::Mat_<double>& t_mat)
{
  double max_val = max(t_mat);

  for (int r = 0; r < t_mat.rows; ++r) {
    for (int c = 0; c < t_mat.cols; ++c) {
      if (std::isnan(t_mat(r, c))) {
        t_mat(r, c) = max_val;
      }
    }
  }
}

bool hiros::track::utils::isEmpty(const hiros::skeletons::types::Skeleton& t_skeleton)
{
  bool is_empty = true;

  for (auto& kpg : t_skeleton.skeleton_parts) {
    if (!kpg.keypoints.empty()) {
      is_empty = false;
      break;
    }
  }

  return is_empty;
}

int hiros::track::utils::numberOfKeypoints(const hiros::skeletons::types::Skeleton& t_skeleton)
{
  int n_kps = 0;

  for (auto& kpg : t_skeleton.skeleton_parts) {
    n_kps += kpg.keypoints.size();
  }

  return n_kps;
}

hiros::skeletons::types::Keypoint hiros::track::utils::findKeypoint(const hiros::skeletons::types::Skeleton& t_skeleton,
                                                                    const int& t_keypoint_group_id,
                                                                    const int& t_keypoint_id)
{
  for (auto& kpg : t_skeleton.skeleton_parts) {
    if (kpg.id == t_keypoint_group_id) {
      for (auto& kp : kpg.keypoints) {
        if (kp.id == t_keypoint_id) {
          return kp;
        }

        if (kp.id > t_keypoint_id) {
          return hiros::skeletons::types::Keypoint();
        }
      }

      return hiros::skeletons::types::Keypoint();
    }

    if (kpg.id > t_keypoint_group_id) {
      return hiros::skeletons::types::Keypoint();
    }
  }

  return hiros::skeletons::types::Keypoint();
}

double hiros::track::utils::distance(const hiros::skeletons::types::Point& t_p1,
                                     const hiros::skeletons::types::Point& t_p2)
{
  double squared_dist = std::pow((t_p1.x - t_p2.x), 2) + std::pow((t_p1.y - t_p2.y), 2);

  if (!std::isnan(t_p1.z) && !std::isnan(t_p2.z)) {
    squared_dist += std::pow((t_p1.z - t_p1.z), 2);
  }

  return std::sqrt(squared_dist);
}
