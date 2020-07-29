// Custom external dependencies
#include "skeletons/utils.h"

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

void hiros::track::utils::replaceNans(cv::Mat_<double>& t_mat)
{
  if (isNaN(t_mat)) {
    t_mat.setTo(std::numeric_limits<double>::max());
  }
  else {
    utils::replaceNansWithMax(t_mat);
  }
}

void hiros::track::utils::minWithIndex(const cv::Mat_<double>& t_mat,
                                       double& t_min,
                                       unsigned int& t_row,
                                       unsigned int& t_col)
{
  t_row = 0;
  t_col = 0;
  t_min = t_mat(static_cast<int>(t_row), static_cast<int>(t_col));

  for (int r = 0; r < t_mat.rows; ++r) {
    for (int c = 0; c < t_mat.cols; ++c) {
      if (t_mat(r, c) < t_min) {
        t_min = t_mat(r, c);
        t_row = static_cast<unsigned int>(r);
        t_col = static_cast<unsigned int>(c);
      }
    }
  }
}

bool hiros::track::utils::isEmpty(const hiros::skeletons::types::Skeleton& t_skeleton)
{
  for (auto& kpg : t_skeleton.skeleton_parts) {
    if (!kpg.second.keypoints.empty()) {
      return false;
    }
  }

  return true;
}

int hiros::track::utils::numberOfKeypoints(const hiros::skeletons::types::Skeleton& t_skeleton)
{
  int n_kps = 0;

  for (auto& kpg : t_skeleton.skeleton_parts) {
    n_kps += kpg.second.keypoints.size();
  }

  return n_kps;
}

void hiros::track::utils::merge(hiros::skeletons::types::Skeleton& t_s1,
                                const skeletons::types::Skeleton& t_s2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  for (auto& s2_kpg : t_s2.skeleton_parts) {
    for (auto& s2_kp : s2_kpg.second.keypoints) {
      merge(t_s1, s2_kpg.second, s2_kp.second, t_w1, t_w2, t_weight_by_confidence);
    }
  }
}

void hiros::track::utils::merge(skeletons::types::Skeleton& t_sk,
                                const skeletons::types::KeypointGroup& t_kpg,
                                const skeletons::types::Keypoint& t_kp,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  if (hiros::skeletons::utils::hasKeypoint(t_sk, t_kpg.id, t_kp.id)) {
    t_sk.skeleton_parts.at(t_kpg.id).keypoints.at(t_kp.id) =
      t_weight_by_confidence ? wavg(t_sk.skeleton_parts.at(t_kpg.id).keypoints.at(t_kp.id),
                                    t_kp,
                                    t_w1 * t_sk.skeleton_parts.at(t_kpg.id).keypoints.at(t_kp.id).confidence,
                                    t_w2 * t_kp.confidence)
                             : wavg(t_sk.skeleton_parts.at(t_kpg.id).keypoints.at(t_kp.id), t_kp, t_w1, t_w2);
  }
  else if (hiros::skeletons::utils::hasKeypointGroup(t_sk, t_kpg.id)) {
    t_sk.skeleton_parts.at(t_kpg.id).keypoints.emplace(t_kp.id, t_kp);
  }
  else {
    t_sk.skeleton_parts.emplace(t_kpg.id, t_kpg);
  }
}

hiros::skeletons::types::Vector hiros::track::utils::wavg(const hiros::skeletons::types::Vector& t_v1,
                                                          const hiros::skeletons::types::Vector& t_v2,
                                                          const double& t_w1,
                                                          const double& t_w2)
{
  return (t_w1 * t_v1 + t_w2 * t_v2) / (t_w1 + t_w2);
}

hiros::skeletons::types::Keypoint hiros::track::utils::wavg(const skeletons::types::Keypoint& t_kp1,
                                                            const skeletons::types::Keypoint& t_kp2,
                                                            const double& t_w1,
                                                            const double& t_w2)
{
  if (t_kp1.id != t_kp2.id) {
    return skeletons::types::Keypoint();
  }

  skeletons::types::Keypoint avg_kp;

  avg_kp.id = t_kp1.id;
  avg_kp.confidence = (t_w1 * t_kp1.confidence + t_w2 * t_kp2.confidence) / (t_w1 + t_w2);
  avg_kp.point.position = wavg(t_kp1.point.position, t_kp2.point.position, t_w1, t_w2);
  avg_kp.point.velocity = wavg(t_kp1.point.velocity, t_kp2.point.velocity, t_w1, t_w2);
  avg_kp.point.acceleration = wavg(t_kp1.point.acceleration, t_kp2.point.acceleration, t_w1, t_w2);

  return avg_kp;
}
