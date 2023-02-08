// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/utils.h"

bool hiros::skeletons::utils::isNaN(const cv::Mat_<double>& mat) {
  for (auto r{0}; r < mat.rows; ++r) {
    for (auto c{0}; c < mat.cols; ++c) {
      if (!std::isnan(mat(r, c))) {
        return false;
      }
    }
  }

  return true;
}

double hiros::skeletons::utils::min(const cv::Mat_<double>& mat) {
  auto min{std::numeric_limits<double>::max()};

  for (auto r{0}; r < mat.rows; ++r) {
    for (auto c{0}; c < mat.cols; ++c) {
      if (!std::isnan(mat(r, c))) {
        min = std::min(mat(r, c), min);
      }
    }
  }

  return min;
}

double hiros::skeletons::utils::max(const cv::Mat_<double>& mat) {
  auto max{-std::numeric_limits<double>::max()};

  for (auto r{0}; r < mat.rows; ++r) {
    for (auto c{0}; c < mat.cols; ++c) {
      if (!std::isnan(mat(r, c))) {
        max = std::max(mat(r, c), max);
      }
    }
  }

  return max;
}

void hiros::skeletons::utils::minWithIndex(const cv::Mat_<double>& mat,
                                           double& min, unsigned int& row,
                                           unsigned int& col) {
  row = 0;
  col = 0;
  min = mat(static_cast<int>(row), static_cast<int>(col));

  for (auto r{0}; r < mat.rows; ++r) {
    for (auto c{0}; c < mat.cols; ++c) {
      if (mat(r, c) < min) {
        min = mat(r, c);
        row = static_cast<unsigned int>(r);
        col = static_cast<unsigned int>(c);
      }
    }
  }
}

bool hiros::skeletons::utils::matchMunkres(const cv::Mat_<int>& munkres_matrix,
                                           const unsigned int& row,
                                           const unsigned int& col) {
  return (munkres_matrix[static_cast<int>(row)][static_cast<int>(col)] == 1);
}

bool hiros::skeletons::utils::isEmpty(
    const hiros::skeletons::types::Skeleton& skeleton) {
  return (skeleton.markers.empty() && skeleton.links.empty());
}

bool hiros::skeletons::utils::isEmpty(
    const hiros_skeleton_msgs::msg::Skeleton& skeleton) {
  return (skeleton.markers.empty() && skeleton.links.empty());
}

bool hiros::skeletons::utils::isEmpty(
    const hiros::skeletons::types::SkeletonGroup& skeleton_group) {
  for (const auto& skeleton : skeleton_group.skeletons) {
    if (!isEmpty(skeleton)) {
      return true;
    }
  }

  return false;
}

bool hiros::skeletons::utils::isEmpty(
    const hiros_skeleton_msgs::msg::SkeletonGroup& skeleton_group) {
  for (const auto& skeleton : skeleton_group.skeletons) {
    if (!isEmpty(skeleton)) {
      return false;
    }
  }

  return true;
}

hiros::skeletons::types::Skeleton hiros::skeletons::utils::predict(
    const hiros::skeletons::types::Skeleton& skeleton,
    const double& current_time) {
  auto dt{current_time - skeleton.src_time};
  tf2::Vector3 delta_theta{};
  tf2::Quaternion delta_q{};

  auto pred_skel{skeleton};
  pred_skel.src_time = current_time;

  for (auto& m : pred_skel.markers) {
    predict(m.center, dt);
  }

  for (auto& l : pred_skel.links) {
    predict(l.center, dt);
  }

  return pred_skel;
}

void hiros::skeletons::utils::predict(
    hiros::skeletons::types::KinematicState& state, const double& dt) {
  if (!isNaN(state.velocity.linear)) {
    state.pose.position += (dt * state.velocity.linear);
  }

  if (!isNaN(state.velocity.angular)) {
    tf2::Quaternion delta_q{};
    auto delta_theta{dt * state.velocity.angular};
    delta_q.setEuler(delta_theta.z(), delta_theta.y(),
                     delta_theta.x());  // yaw, pitch, roll
    state.pose.orientation = delta_q * state.pose.orientation;
  }
}

void hiros::skeletons::utils::merge(hiros::skeletons::types::Skeleton& s1,
                                    const skeletons::types::Skeleton& s2,
                                    const double& w1, const double& w2,
                                    const bool& weight_by_confidence) {
  s1.src_time = wavg(s1.src_time, s2.src_time, w1, w2);

  for (auto& s2_mk : s2.markers) {
    merge(s1, s2_mk, w1, w2, weight_by_confidence);
  }

  for (auto& s2_lk : s2.links) {
    merge(s1, s2_lk, w1, w2, weight_by_confidence);
  }

  s1.bounding_box = skeletons::utils::computeBoundingBox(s1);
}

void hiros::skeletons::utils::merge(skeletons::types::Skeleton& sk1,
                                    const skeletons::types::Marker& mk2,
                                    const double& w1, const double& w2,
                                    const bool& weight_by_confidence) {
  if (sk1.hasMarker(mk2.id)) {
    auto& mk1{sk1.getMarker(mk2.id)};
    mk1 = weight_by_confidence
              ? wavg(mk1, mk2, w1 * mk1.confidence, w2 * mk2.confidence)
              : wavg(mk1, mk2, w1, w2);
  } else {
    sk1.addMarker(mk2);
  }
}

void hiros::skeletons::utils::merge(skeletons::types::Skeleton& sk1,
                                    const skeletons::types::Link& lk2,
                                    const double& w1, const double& w2,
                                    const bool& weight_by_confidence) {
  if (sk1.hasLink(lk2.id)) {
    auto& lk1{sk1.getLink(lk2.id)};
    lk1 = weight_by_confidence
              ? wavg(lk1, lk2, w1 * lk1.confidence, w2 * lk2.confidence)
              : wavg(lk1, lk2, w1, w2);
  } else {
    sk1.addLink(lk2);
  }

  // Fix link center if possible
  if (sk1.hasMarker(lk2.parent_marker) && sk1.hasMarker(lk2.child_marker)) {
    sk1.getLink(lk2.id).center.pose.position =
        wavg(sk1.getMarker(lk2.parent_marker).center.pose.position,
             sk1.getMarker(lk2.child_marker).center.pose.position);
  }

  // Align link orientation if possible
  alignLinkOrientation(sk1, lk2.id);
}

void hiros::skeletons::utils::alignLinkOrientation(
    hiros::skeletons::types::Skeleton& sk, const int& lk_id) {
  if (!sk.hasLink(lk_id)) {
    return;
  }

  auto& link{sk.getLink(lk_id)};

  if (skeletons::utils::isNaN(link.center.pose.orientation) ||
      !sk.hasMarker(link.parent_marker) || !sk.hasMarker(link.child_marker)) {
    return;
  }

  auto link_axis{(sk.getMarker(link.parent_marker).center.pose.position -
                  sk.getMarker(link.child_marker).center.pose.position)
                     .normalized()};

  auto closest_cartesian_axis{closestCartesianAxis(
      tf2::quatRotate(link.center.pose.orientation.inverse(), link_axis)
          .normalized())};

  // Axis of the link orientation SoR to be aligned to the link axis
  auto quat_axis{
      tf2::quatRotate(link.center.pose.orientation, closest_cartesian_axis)
          .normalized()};

  auto rot_axis{quat_axis.cross(link_axis).normalized()};
  auto rot_angle{acos(std::min(std::max(-1., quat_axis.dot(link_axis)), 1.))};
  // Rotation to align the link orientation to the link axis
  auto rot_quat{tf2::Quaternion(rot_axis, rot_angle)};

  link.center.pose.orientation = rot_quat * link.center.pose.orientation;
}

tf2::Vector3 hiros::skeletons::utils::closestCartesianAxis(
    const tf2::Vector3& vec) {
  auto closest_axis_idx{vec.closestAxis()};  // 0: x, 1: y, 2: z

  tf2::Vector3DoubleData signs{};
  signs.m_floats[0] = vec.x() >= 0 ? 1 : -1;
  signs.m_floats[1] = vec.y() >= 0 ? 1 : -1;
  signs.m_floats[2] = vec.z() >= 0 ? 1 : -1;

  tf2::Vector3DoubleData closest_axis_serialized{0, 0, 0};
  closest_axis_serialized.m_floats[closest_axis_idx] =
      signs.m_floats[closest_axis_idx];

  tf2::Vector3 closest_axis{};
  closest_axis.deSerialize(closest_axis_serialized);

  return closest_axis;
}

double hiros::skeletons::utils::wavg(const double& e1, const double& e2,
                                     const double& w1, const double& w2) {
  auto weight{(w1 + w2 != 0.) ? w2 / (w1 + w2) : 0.5};
  return (1 - weight) * e1 + weight * e2;
}

tf2::Vector3 hiros::skeletons::utils::wavg(const tf2::Vector3& v1,
                                           const tf2::Vector3& v2,
                                           const double& w1, const double& w2) {
  auto weight{(w1 + w2 != 0.) ? w2 / (w1 + w2) : 0.5};
  return v1.lerp(v2, weight);
}

tf2::Quaternion hiros::skeletons::utils::wavg(const tf2::Quaternion& q1,
                                              const tf2::Quaternion& q2,
                                              const double& w1,
                                              const double& w2) {
  auto weight{(w1 + w2 != 0.) ? w2 / (w1 + w2) : 0.5};
  return q1.normalized().slerp(q2.normalized(), weight).normalize();
}

hiros::skeletons::types::KinematicState hiros::skeletons::utils::wavg(
    const skeletons::types::KinematicState& ks1,
    const skeletons::types::KinematicState& ks2, const double& w1,
    const double& w2) {
  skeletons::types::KinematicState avg_ks{};

  avg_ks.pose.position = wavg(ks1.pose.position, ks2.pose.position, w1, w2);
  avg_ks.pose.orientation =
      wavg(ks1.pose.orientation, ks2.pose.orientation, w1, w2);
  avg_ks.velocity.linear =
      wavg(ks1.velocity.linear, ks2.velocity.linear, w1, w2);
  avg_ks.velocity.angular =
      wavg(ks1.velocity.angular, ks2.velocity.angular, w1, w2);
  avg_ks.acceleration.linear =
      wavg(ks1.acceleration.linear, ks2.acceleration.linear, w1, w2);
  avg_ks.acceleration.angular =
      wavg(ks1.acceleration.angular, ks2.acceleration.angular, w1, w2);

  return avg_ks;
}

hiros::skeletons::types::Marker hiros::skeletons::utils::wavg(
    const skeletons::types::Marker& mk1, const skeletons::types::Marker& mk2,
    const double& w1, const double& w2) {
  if (mk1.id != mk2.id) {
    std::cerr << "Warning: trying to average different markers" << std::endl;
    return skeletons::types::Marker();
  }

  auto avg_mk{mk1};

  avg_mk.confidence =
      (w1 + w2 != 0.) ? (w1 * mk1.confidence + w2 * mk2.confidence) / (w1 + w2)
                      : 0.;
  avg_mk.center = wavg(mk1.center, mk2.center, w1, w2);

  return avg_mk;
}

hiros::skeletons::types::Link hiros::skeletons::utils::wavg(
    const skeletons::types::Link& lk1, const skeletons::types::Link& lk2,
    const double& w1, const double& w2) {
  if (lk1.id != lk2.id) {
    std::cerr << "Warning: trying to average different links" << std::endl;
    return skeletons::types::Link();
  }

  auto avg_lk{lk1};

  avg_lk.confidence =
      (w1 + w2 != 0.) ? (w1 * lk1.confidence + w2 * lk2.confidence) / (w1 + w2)
                      : 0.;
  avg_lk.center = wavg(lk1.center, lk2.center, w1, w2);

  return avg_lk;
}

rclcpp::Time hiros::skeletons::utils::avgSrcTime(
    const hiros::skeletons::types::SkeletonGroup& skel_group) {
  if (skel_group.skeletons.empty()) {
    return rclcpp::Time();
  }

  auto sum{0.};
  auto n_elems{0u};
  for (const auto& skel : skel_group.skeletons) {
    sum += skel.src_time;
    ++n_elems;
  }

  return rclcpp::Time{static_cast<long>(sum / n_elems * 1e9)};
}

rclcpp::Time hiros::skeletons::utils::avgSrcTime(
    const hiros_skeleton_msgs::msg::SkeletonGroup& skel_group) {
  return avgSrcTime(toStruct(skel_group));
}

rclcpp::Time hiros::skeletons::utils::oldestSrcTime(
    const hiros::skeletons::types::SkeletonGroup& skel_group) {
  if (skel_group.skeletons.empty()) {
    return rclcpp::Time();
  }

  return rclcpp::Time{static_cast<long>(
      std::min_element(skel_group.skeletons.begin(), skel_group.skeletons.end(),
                       [](const auto& lhs, const auto& rhs) {
                         return lhs.src_time < rhs.src_time;
                       })
          ->src_time *
      1e9)};
}

rclcpp::Time hiros::skeletons::utils::oldestSrcTime(
    const hiros_skeleton_msgs::msg::SkeletonGroup& skel_group) {
  return oldestSrcTime(toStruct(skel_group));
}

rclcpp::Time hiros::skeletons::utils::newestSrcTime(
    const hiros::skeletons::types::SkeletonGroup& skel_group) {
  if (skel_group.skeletons.empty()) {
    return rclcpp::Time();
  }

  return rclcpp::Time{static_cast<long>(
      std::max_element(skel_group.skeletons.begin(), skel_group.skeletons.end(),
                       [](const auto& lhs, const auto& rhs) {
                         return lhs.src_time < rhs.src_time;
                       })
          ->src_time *
      1e9)};
}

rclcpp::Time hiros::skeletons::utils::newestSrcTime(
    const hiros_skeleton_msgs::msg::SkeletonGroup& skel_group) {
  return newestSrcTime(toStruct(skel_group));
}
