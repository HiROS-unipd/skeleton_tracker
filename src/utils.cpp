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
  return skeletons::utils::numberOfMarkers(t_skeleton) + skeletons::utils::numberOfOrientations(t_skeleton) == 0;
}

bool hiros::track::utils::isEmpty(const hiros_skeleton_msgs::Skeleton& t_skeleton)
{
  for (auto& mkg : t_skeleton.marker_groups) {
    if (!mkg.markers.empty()) {
      return false;
    }
  }

  for (auto& org : t_skeleton.orientation_groups) {
    if (!org.orientations.empty()) {
      return false;
    }
  }

  return true;
}

hiros::skeletons::types::Skeleton hiros::track::utils::predict(const hiros::skeletons::types::Skeleton& t_track,
                                                               const double& t_current_time)
{
  double dt = t_current_time - t_track.src_time;

  auto pred_track = t_track;
  pred_track.src_time = t_current_time;

  for (auto& mg : pred_track.marker_groups) {
    for (auto& m : mg.markers) {
      m.point.position += (dt * m.point.velocity);
    }
  }

  tf2::Vector3 delta_theta;
  tf2::Quaternion delta_q;
  for (auto& og : pred_track.orientation_groups) {
    for (auto& o : og.orientations) {
      delta_theta = dt * o.mimu.angular_velocity;
      delta_q.setEuler(delta_theta.z(), delta_theta.y(), delta_theta.x()); // yaw, pitch, roll
      o.mimu.orientation = delta_q * o.mimu.orientation;
    }
  }

  return pred_track;
}

void hiros::track::utils::merge(hiros::skeletons::types::Skeleton& t_s1,
                                const skeletons::types::Skeleton& t_s2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  for (auto& s2_mkg : t_s2.marker_groups) {
    for (auto& s2_mk : s2_mkg.markers) {
      merge(t_s1, s2_mkg, s2_mk, t_w1, t_w2, t_weight_by_confidence);
    }
  }

  for (auto& s2_org : t_s2.orientation_groups) {
    for (auto& s2_or : s2_org.orientations) {
      merge(t_s1, s2_org, s2_or, t_w1, t_w2, t_weight_by_confidence);
    }
  }
}

void hiros::track::utils::merge(skeletons::types::Skeleton& t_sk1,
                                const skeletons::types::MarkerGroup& t_mkg2,
                                const skeletons::types::Marker& t_mk2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  if (t_sk1.hasMarkerGroup(t_mkg2.id)) {
    auto& mkg1 = t_sk1.getMarkerGroup(t_mkg2.id);
    if (mkg1.hasMarker(t_mk2.id)) {
      auto& mk1 = mkg1.getMarker(t_mk2.id);
      mk1 = t_weight_by_confidence ? wavg(mk1, t_mk2, t_w1 * mk1.confidence, t_w2 * t_mk2.confidence)
                                   : wavg(mk1, t_mk2, t_w1, t_w2);
    }
    else {
      mkg1.addMarker(t_mk2);
    }
  }
  else {
    t_sk1.addMarkerGroup(t_mkg2);
  }
}

void hiros::track::utils::merge(skeletons::types::Skeleton& t_sk1,
                                const skeletons::types::OrientationGroup& t_org2,
                                const skeletons::types::Orientation& t_or2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  if (t_sk1.hasOrientationGroup(t_org2.id)) {
    auto& org1 = t_sk1.getOrientationGroup(t_org2.id);
    if (org1.hasOrientation(t_or2.id)) {
      auto& or1 = org1.getOrientation(t_or2.id);
      or1 = t_weight_by_confidence ? wavg(or1, t_or2, t_w1 * or1.confidence, t_w2 * t_or2.confidence)
                                   : wavg(or1, t_or2, t_w1, t_w2);
    }
    else {
      org1.addOrientation(t_or2);
    }
  }
  else {
    t_sk1.addOrientationGroup(t_org2);
  }
}

tf2::Vector3
hiros::track::utils::wavg(const tf2::Vector3& t_v1, const tf2::Vector3& t_v2, const double& t_w1, const double& t_w2)
{
  auto weight = (t_w1 + t_w2 != 0.) ? t_w2 / (t_w1 + t_w2) : 0.5;
  return t_v1.lerp(t_v2, weight);
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

  skeletons::types::Marker avg_mk;

  avg_mk.id = t_mk1.id;
  avg_mk.confidence = (t_w1 + t_w2 != 0.) ? (t_w1 * t_mk1.confidence + t_w2 * t_mk2.confidence) / (t_w1 + t_w2) : 0.;
  avg_mk.point.position = wavg(t_mk1.point.position, t_mk2.point.position, t_w1, t_w2);
  avg_mk.point.velocity = wavg(t_mk1.point.velocity, t_mk2.point.velocity, t_w1, t_w2);
  avg_mk.point.acceleration = wavg(t_mk1.point.acceleration, t_mk2.point.acceleration, t_w1, t_w2);

  return avg_mk;
}

tf2::Quaternion hiros::track::utils::wavg(const tf2::Quaternion& t_q1,
                                          const tf2::Quaternion& t_q2,
                                          const double& t_w1,
                                          const double& t_w2)
{
  auto weight = (t_w1 + t_w2 != 0.) ? t_w2 / (t_w1 + t_w2) : 0.5;
  return t_q1.normalized().slerp(t_q2.normalized(), weight).normalize();
}

hiros::skeletons::types::Orientation hiros::track::utils::wavg(const skeletons::types::Orientation& t_or1,
                                                               const skeletons::types::Orientation& t_or2,
                                                               const double& t_w1,
                                                               const double& t_w2)
{
  if (t_or1.id != t_or2.id) {
    std::cerr << "Warning: trying to average different orientations" << std::endl;
    return skeletons::types::Orientation();
  }

  skeletons::types::Orientation avg_or;

  avg_or.id = t_or1.id;
  avg_or.confidence = (t_w1 + t_w2 != 0.) ? (t_w1 * t_or1.confidence + t_w2 * t_or2.confidence) / (t_w1 + t_w2) : 0.;
  avg_or.mimu.frame_id = t_or1.mimu.frame_id;
  avg_or.mimu.orientation = wavg(t_or1.mimu.orientation, t_or2.mimu.orientation, t_w1, t_w2);
  avg_or.mimu.angular_velocity = wavg(t_or1.mimu.angular_velocity, t_or2.mimu.angular_velocity, t_w1, t_w2);
  avg_or.mimu.linear_acceleration = wavg(t_or1.mimu.linear_acceleration, t_or2.mimu.linear_acceleration, t_w1, t_w2);
  avg_or.mimu.magnetic_field = wavg(t_or1.mimu.magnetic_field, t_or2.mimu.magnetic_field, t_w1, t_w2);

  return avg_or;
}

ros::Time hiros::track::utils::avgSrcTime(const hiros::skeletons::types::SkeletonGroup& t_skel_group)
{
  if (t_skel_group.skeletons.empty()) {
    return ros::Time();
  }

  double sum{0};
  unsigned int n_elems{0};
  for (const auto& skel : t_skel_group.skeletons) {
    if (!isEmpty(skel)) {
      sum += skel.src_time;
      ++n_elems;
    }
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
