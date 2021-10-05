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

double hiros::track::utils::min(const cv::Mat_<double>& t_mat)
{
  double min = std::numeric_limits<double>::max();

  for (int r = 0; r < t_mat.rows; ++r) {
    for (int c = 0; c < t_mat.cols; ++c) {
      if (!std::isnan(t_mat(r, c))) {
        min = std::min(t_mat(r, c), min);
      }
    }
  }

  return min;
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

bool hiros::track::utils::matchMunkres(const cv::Mat_<int>& t_munkres_matrix,
                                       const unsigned int& t_row,
                                       const unsigned int& t_col)
{
  return (t_munkres_matrix[static_cast<int>(t_row)][static_cast<int>(t_col)] == 1);
}

bool hiros::track::utils::isEmpty(const hiros::skeletons::types::Skeleton& t_skeleton)
{
  return (t_skeleton.markers.empty() && t_skeleton.links.empty());
}

bool hiros::track::utils::isEmpty(const hiros_skeleton_msgs::Skeleton& t_skeleton)
{
  return (t_skeleton.markers.empty() && t_skeleton.links.empty());
}

bool hiros::track::utils::isEmpty(const hiros::skeletons::types::SkeletonGroup& t_skeleton_group)
{
  for (const auto& skeleton : t_skeleton_group.skeletons) {
    if (!isEmpty(skeleton)) {
      return true;
    }
  }

  return false;
}

bool hiros::track::utils::isEmpty(const hiros_skeleton_msgs::SkeletonGroup& t_skeleton_group)
{
  for (const auto& skeleton : t_skeleton_group.skeletons) {
    if (!isEmpty(skeleton)) {
      return false;
    }
  }

  return true;
}

hiros::skeletons::types::Skeleton hiros::track::utils::predict(const hiros::skeletons::types::Skeleton& t_skeleton,
                                                               const double& t_current_time)
{
  double dt = t_current_time - t_skeleton.src_time;
  tf2::Vector3 delta_theta;
  tf2::Quaternion delta_q;

  auto pred_skel = t_skeleton;
  pred_skel.src_time = t_current_time;

  for (auto& m : pred_skel.markers) {
    predict(m.center, dt);
  }

  for (auto& l : pred_skel.links) {
    predict(l.center, dt);
  }

  return pred_skel;
}

void hiros::track::utils::predict(hiros::skeletons::types::KinematicState& t_state, const double& t_dt)
{
  t_state.pose.position += (t_dt * t_state.velocity.linear);

  tf2::Quaternion delta_q;
  auto delta_theta = t_dt * t_state.velocity.angular;
  delta_q.setEuler(delta_theta.z(), delta_theta.y(), delta_theta.x()); // yaw, pitch, roll
  t_state.pose.orientation = delta_q * t_state.pose.orientation;
}

void hiros::track::utils::merge(hiros::skeletons::types::Skeleton& t_s1,
                                const skeletons::types::Skeleton& t_s2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  t_s1.src_time = wavg(t_s1.src_time, t_s2.src_time, t_w1, t_w2);

  for (auto& s2_mk : t_s2.markers) {
    merge(t_s1, s2_mk, t_w1, t_w2, t_weight_by_confidence);
  }

  for (auto& s2_lk : t_s2.links) {
    merge(t_s1, s2_lk, t_w1, t_w2, t_weight_by_confidence);
  }
}

void hiros::track::utils::merge(skeletons::types::Skeleton& t_sk1,
                                const skeletons::types::Marker& t_mk2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  if (t_sk1.hasMarker(t_mk2.id)) {
    auto& mk1 = t_sk1.getMarker(t_mk2.id);
    mk1 = t_weight_by_confidence ? wavg(mk1, t_mk2, t_w1 * mk1.confidence, t_w2 * t_mk2.confidence)
                                 : wavg(mk1, t_mk2, t_w1, t_w2);
  }
  else {
    t_sk1.addMarker(t_mk2);
  }
}

void hiros::track::utils::merge(skeletons::types::Skeleton& t_sk1,
                                const skeletons::types::Link& t_lk2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  if (t_sk1.hasLink(t_lk2.id)) {
    auto& lk1 = t_sk1.getLink(t_lk2.id);
    lk1 = t_weight_by_confidence ? wavg(lk1, t_lk2, t_w1 * lk1.confidence, t_w2 * t_lk2.confidence)
                                 : wavg(lk1, t_lk2, t_w1, t_w2);
  }
  else {
    t_sk1.addLink(t_lk2);
  }
}

double hiros::track::utils::wavg(const double& t_e1, const double& t_e2, const double& t_w1, const double& t_w2)
{
  auto weight = (t_w1 + t_w2 != 0.) ? t_w2 / (t_w1 + t_w2) : 0.5;
  return (1 - weight) * t_e1 + weight * t_e2;
}

tf2::Vector3
hiros::track::utils::wavg(const tf2::Vector3& t_v1, const tf2::Vector3& t_v2, const double& t_w1, const double& t_w2)
{
  auto weight = (t_w1 + t_w2 != 0.) ? t_w2 / (t_w1 + t_w2) : 0.5;
  return t_v1.lerp(t_v2, weight);
}

tf2::Quaternion hiros::track::utils::wavg(const tf2::Quaternion& t_q1,
                                          const tf2::Quaternion& t_q2,
                                          const double& t_w1,
                                          const double& t_w2)
{
  auto weight = (t_w1 + t_w2 != 0.) ? t_w2 / (t_w1 + t_w2) : 0.5;
  return t_q1.normalized().slerp(t_q2.normalized(), weight).normalize();
}

hiros::skeletons::types::KinematicState hiros::track::utils::wavg(const skeletons::types::KinematicState& t_ks1,
                                                                  const skeletons::types::KinematicState& t_ks2,
                                                                  const double& t_w1,
                                                                  const double& t_w2)
{
  skeletons::types::KinematicState avg_ks;

  avg_ks.pose.position = wavg(t_ks1.pose.position, t_ks2.pose.position, t_w1, t_w2);
  avg_ks.pose.orientation = wavg(t_ks1.pose.orientation, t_ks2.pose.orientation, t_w1, t_w2);
  avg_ks.velocity.linear = wavg(t_ks1.velocity.linear, t_ks2.velocity.linear, t_w1, t_w2);
  avg_ks.velocity.angular = wavg(t_ks1.velocity.angular, t_ks2.velocity.angular, t_w1, t_w2);
  avg_ks.acceleration.linear = wavg(t_ks1.acceleration.linear, t_ks2.acceleration.linear, t_w1, t_w2);
  avg_ks.acceleration.angular = wavg(t_ks1.acceleration.angular, t_ks2.acceleration.angular, t_w1, t_w2);

  return avg_ks;
}

hiros::skeletons::types::Marker hiros::track::utils::wavg(const skeletons::types::Marker& t_mk1,
                                                          const skeletons::types::Marker& t_mk2,
                                                          const double& t_w1,
                                                          const double& t_w2)
{
  if (t_mk1.id != t_mk2.id) {
    std::cerr << "Warning: trying to average different markers" << std::endl;
    return skeletons::types::Marker();
  }

  skeletons::types::Marker avg_mk(t_mk1);

  avg_mk.confidence = (t_w1 + t_w2 != 0.) ? (t_w1 * t_mk1.confidence + t_w2 * t_mk2.confidence) / (t_w1 + t_w2) : 0.;
  avg_mk.center = wavg(t_mk1.center, t_mk2.center, t_w1, t_w2);

  return avg_mk;
}

hiros::skeletons::types::Link hiros::track::utils::wavg(const skeletons::types::Link& t_lk1,
                                                        const skeletons::types::Link& t_lk2,
                                                        const double& t_w1,
                                                        const double& t_w2)
{
  if (t_lk1.id != t_lk2.id) {
    std::cerr << "Warning: trying to average different links" << std::endl;
    return skeletons::types::Link();
  }

  skeletons::types::Link avg_lk(t_lk1);

  avg_lk.confidence = (t_w1 + t_w2 != 0.) ? (t_w1 * t_lk1.confidence + t_w2 * t_lk2.confidence) / (t_w1 + t_w2) : 0.;
  avg_lk.center = wavg(t_lk1.center, t_lk2.center, t_w1, t_w2);

  return avg_lk;
}

ros::Time hiros::track::utils::avgSrcTime(const hiros::skeletons::types::SkeletonGroup& t_skel_group)
{
  if (t_skel_group.skeletons.empty()) {
    return ros::Time();
  }

  double sum{0};
  unsigned int n_elems{0};
  for (const auto& skel : t_skel_group.skeletons) {
    sum += skel.src_time;
    ++n_elems;
  }

  return ros::Time(sum / n_elems);
}

ros::Time hiros::track::utils::avgSrcTime(const hiros_skeleton_msgs::SkeletonGroup& t_skel_group)
{
  return avgSrcTime(hiros::skeletons::utils::toStruct(t_skel_group));
}

ros::Time hiros::track::utils::oldestSrcTime(const hiros::skeletons::types::SkeletonGroup& t_skel_group)
{
  if (t_skel_group.skeletons.empty()) {
    return ros::Time();
  }

  return ros::Time(std::min_element(t_skel_group.skeletons.begin(),
                                    t_skel_group.skeletons.end(),
                                    [](const auto& lhs, const auto& rhs) { return lhs.src_time < rhs.src_time; })
                     ->src_time);
}

ros::Time hiros::track::utils::oldestSrcTime(const hiros_skeleton_msgs::SkeletonGroup& t_skel_group)
{
  return oldestSrcTime(hiros::skeletons::utils::toStruct(t_skel_group));
}

ros::Time hiros::track::utils::newestSrcTime(const hiros::skeletons::types::SkeletonGroup& t_skel_group)
{
  if (t_skel_group.skeletons.empty()) {
    return ros::Time();
  }

  return ros::Time(std::max_element(t_skel_group.skeletons.begin(),
                                    t_skel_group.skeletons.end(),
                                    [](const auto& lhs, const auto& rhs) { return lhs.src_time < rhs.src_time; })
                     ->src_time);
}

ros::Time hiros::track::utils::newestSrcTime(const hiros_skeleton_msgs::SkeletonGroup& t_skel_group)
{
  return newestSrcTime(hiros::skeletons::utils::toStruct(t_skel_group));
}
