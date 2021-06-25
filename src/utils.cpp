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

bool hiros::track::utils::isEmpty(const hiros::skeletons::types::MarkerSkeleton& t_skeleton)
{
  for (auto& mkg : t_skeleton.marker_groups) {
    if (!mkg.second.markers.empty()) {
      return false;
    }
  }

  return true;
}

bool hiros::track::utils::isEmpty(const hiros_skeleton_msgs::MarkerSkeleton& t_skeleton)
{
  for (auto& mkg : t_skeleton.marker_groups) {
    if (!mkg.markers.empty()) {
      return false;
    }
  }

  return true;
}

void hiros::track::utils::merge(hiros::skeletons::types::MarkerSkeleton& t_s1,
                                const skeletons::types::MarkerSkeleton& t_s2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  for (auto& s2_mkg : t_s2.marker_groups) {
    for (auto& s2_mk : s2_mkg.second.markers) {
      merge(t_s1, s2_mkg.second, s2_mk.second, t_w1, t_w2, t_weight_by_confidence);
    }
  }
}

void hiros::track::utils::merge(skeletons::types::MarkerSkeleton& t_sk,
                                const skeletons::types::MarkerGroup& t_mkg,
                                const skeletons::types::Marker& t_mk,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  if (hiros::skeletons::utils::hasMarker(t_sk, t_mkg.id, t_mk.id)) {
    t_sk.marker_groups.at(t_mkg.id).markers.at(t_mk.id) =
      t_weight_by_confidence ? wavg(t_sk.marker_groups.at(t_mkg.id).markers.at(t_mk.id),
                                    t_mk,
                                    t_w1 * t_sk.marker_groups.at(t_mkg.id).markers.at(t_mk.id).confidence,
                                    t_w2 * t_mk.confidence)
                             : wavg(t_sk.marker_groups.at(t_mkg.id).markers.at(t_mk.id), t_mk, t_w1, t_w2);
  }
  else if (hiros::skeletons::utils::hasMarkerGroup(t_sk, t_mkg.id)) {
    t_sk.marker_groups.at(t_mkg.id).addMarker(t_mk);
  }
  else {
    t_sk.addMarkerGroup(t_mkg);
  }
}

tf2::Vector3
hiros::track::utils::wavg(const tf2::Vector3& t_v1, const tf2::Vector3& t_v2, const double& t_w1, const double& t_w2)
{
  return t_v1.lerp(t_v2, t_w2 / (t_w1 + t_w2));
}

hiros::skeletons::types::Marker hiros::track::utils::wavg(const skeletons::types::Marker& t_mk1,
                                                          const skeletons::types::Marker& t_mk2,
                                                          const double& t_w1,
                                                          const double& t_w2)
{
  if (t_mk1.id != t_mk2.id) {
    return skeletons::types::Marker();
  }

  skeletons::types::Marker avg_mk;

  avg_mk.id = t_mk1.id;
  avg_mk.confidence = (t_w1 * t_mk1.confidence + t_w2 * t_mk2.confidence) / (t_w1 + t_w2);
  avg_mk.point.position = wavg(t_mk1.point.position, t_mk2.point.position, t_w1, t_w2);
  avg_mk.point.velocity = wavg(t_mk1.point.velocity, t_mk2.point.velocity, t_w1, t_w2);
  avg_mk.point.acceleration = wavg(t_mk1.point.acceleration, t_mk2.point.acceleration, t_w1, t_w2);

  return avg_mk;
}

bool hiros::track::utils::isEmpty(const hiros::skeletons::types::OrientationSkeleton& t_skeleton)
{
  for (auto& org : t_skeleton.orientation_groups) {
    if (!org.second.orientations.empty()) {
      return false;
    }
  }

  return true;
}

bool hiros::track::utils::isEmpty(const hiros_skeleton_msgs::OrientationSkeleton& t_skeleton)
{
  for (auto& org : t_skeleton.orientation_groups) {
    if (!org.orientations.empty()) {
      return false;
    }
  }

  return true;
}

void hiros::track::utils::merge(hiros::skeletons::types::OrientationSkeleton& t_s1,
                                const hiros::skeletons::types::OrientationSkeleton& t_s2,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  for (auto& s2_org : t_s2.orientation_groups) {
    for (auto& s2_or : s2_org.second.orientations) {
      merge(t_s1, s2_org.second, s2_or.second, t_w1, t_w2, t_weight_by_confidence);
    }
  }
}

void hiros::track::utils::merge(hiros::skeletons::types::OrientationSkeleton& t_sk,
                                const hiros::skeletons::types::OrientationGroup& t_org,
                                const hiros::skeletons::types::Orientation& t_or,
                                const double& t_w1,
                                const double& t_w2,
                                const bool& t_weight_by_confidence)
{
  if (hiros::skeletons::utils::hasOrientation(t_sk, t_org.id, t_or.id)) {
    t_sk.orientation_groups.at(t_org.id).orientations.at(t_or.id) =
      t_weight_by_confidence ? wavg(t_sk.orientation_groups.at(t_org.id).orientations.at(t_or.id),
                                    t_or,
                                    t_w1 * t_sk.orientation_groups.at(t_org.id).orientations.at(t_or.id).confidence,
                                    t_w2 * t_or.confidence)
                             : wavg(t_sk.orientation_groups.at(t_org.id).orientations.at(t_or.id), t_or, t_w1, t_w2);
  }
  else if (hiros::skeletons::utils::hasOrientationGroup(t_sk, t_org.id)) {
    t_sk.orientation_groups.at(t_org.id).addOrientation(t_or);
  }
  else {
    t_sk.addOrientationGroup(t_org);
  }
}

tf2::Quaternion hiros::track::utils::wavg(const tf2::Quaternion& t_q1,
                                          const tf2::Quaternion& t_q2,
                                          const double& t_w1,
                                          const double& t_w2)
{
  return t_q1.normalized().slerp(t_q2.normalized(), t_w2 / (t_w1 + t_w2)).normalize();
}

hiros::skeletons::types::Orientation hiros::track::utils::wavg(const skeletons::types::Orientation& t_or1,
                                                               const skeletons::types::Orientation& t_or2,
                                                               const double& t_w1,
                                                               const double& t_w2)
{
  if (t_or1.id != t_or2.id) {
    return skeletons::types::Orientation();
  }

  skeletons::types::Orientation avg_or;

  avg_or.id = t_or1.id;
  avg_or.confidence = (t_w1 * t_or1.confidence + t_w2 * t_or2.confidence) / (t_w1 + t_w2);
  avg_or.mimu.frame_id = t_or1.mimu.frame_id;
  avg_or.mimu.orientation = wavg(t_or1.mimu.orientation, t_or2.mimu.orientation, t_w1, t_w2);
  avg_or.mimu.angular_velocity = wavg(t_or1.mimu.angular_velocity, t_or2.mimu.angular_velocity, t_w1, t_w2);
  avg_or.mimu.linear_acceleration = wavg(t_or1.mimu.linear_acceleration, t_or2.mimu.linear_acceleration, t_w1, t_w2);
  avg_or.mimu.magnetic_field = wavg(t_or1.mimu.magnetic_field, t_or2.mimu.magnetic_field, t_w1, t_w2);

  return avg_or;
}
