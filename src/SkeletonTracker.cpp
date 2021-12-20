// Standard dependencies
#include <numeric>

// ROS dependencies
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Custom External Packages dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/SkeletonTracker.h"
#include "skeleton_tracker/utils.h"

hiros::track::SkeletonTracker::SkeletonTracker()
  : m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_last_track_id(-1)
  , m_configured(false)
{}

hiros::track::SkeletonTracker::~SkeletonTracker() {}

void hiros::track::SkeletonTracker::configure()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker... Configuring");

  if (m_configured) {
    m_configured = false;
    stop();
  }

  m_nh.getParam("node_name", m_params.node_name);
  m_nh.getParam("in_skeleton_group_topics", m_params.in_skeleton_group_topics);
  m_nh.getParam("out_msg_topic_name", m_params.out_msg_topic_name);

  if (m_params.in_skeleton_group_topics.empty() || m_params.out_msg_topic_name.empty()) {
    ROS_FATAL_STREAM("Hi-ROS Skeleton Tracker Error: Required topics configuration not provided. Unable to continue");
    ros::shutdown();
  }

  m_n_detectors = m_params.in_skeleton_group_topics.size();
  double max_delta_t = 0, fixed_delay = 0;

  m_nh.getParam("fixed_delay", fixed_delay);
  m_params.fixed_delay = ros::Duration(fixed_delay);
  m_nh.getParam("min_skeleton_confidence", m_params.min_skeleton_confidence);
  m_nh.getParam("min_marker_confidence", m_params.min_marker_confidence);
  m_nh.getParam("min_link_confidence", m_params.min_link_confidence);
  m_nh.getParam("min_markers", m_params.min_markers);
  m_nh.getParam("min_links", m_params.min_links);
  m_nh.getParam("min_linear_distance", m_params.min_linear_distance);
  if (m_params.min_linear_distance <= 0) {
    m_params.min_linear_distance = std::numeric_limits<double>::max();
  }
  m_nh.getParam("max_linear_distance", m_params.max_linear_distance);
  if (m_params.max_linear_distance <= 0) {
    m_params.max_linear_distance = std::numeric_limits<double>::max();
  }
  m_nh.getParam("min_angular_distance", m_params.min_angular_distance);
  if (m_params.min_angular_distance <= 0) {
    m_params.min_angular_distance = std::numeric_limits<double>::max();
  }
  m_nh.getParam("max_angular_distance", m_params.max_angular_distance);
  if (m_params.max_angular_distance <= 0) {
    m_params.max_angular_distance = std::numeric_limits<double>::max();
  }
  m_nh.getParam("max_delta_t", max_delta_t);
  m_params.max_delta_t = ros::Duration(max_delta_t);
  m_nh.getParam("use_positions", m_params.use_positions);
  m_nh.getParam("use_linear_velocities", m_params.use_linear_velocities);
  m_nh.getParam("use_orientations", m_params.use_orientations);
  m_nh.getParam("use_angular_velocities", m_params.use_angular_velocities);
  m_nh.getParam("velocity_weight", m_params.velocity_weight);
  m_nh.getParam("weight_distances_by_confidences", m_params.weight_distances_by_confidences);
  m_nh.getParam("weight_distances_by_velocities", m_params.weight_distances_by_velocities);

  if (!m_params.use_positions && !m_params.use_orientations && !m_params.use_linear_velocities
      && !m_params.use_angular_velocities) {
    ROS_FATAL_STREAM("Hi-ROS Skeleton Tracker Error: Linear/angular distances and/or velocity based distances must be "
                     "enabled. Unable to continue");
    ros::shutdown();
  }

  setupRosTopics();

  m_configured = true;
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker... CONFIGURED" << BASH_MSG_RESET);
}

void hiros::track::SkeletonTracker::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker... Starting");

  if (!m_configured) {
    configure();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker... RUNNING" << BASH_MSG_RESET);
  ros::spin();
}

void hiros::track::SkeletonTracker::stop()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker... Stopping");

  if (!m_in_skeleton_group_subs.empty()) {
    for (auto& sub : m_in_skeleton_group_subs)
      sub.shutdown();
  }

  if (m_out_msg_pub) {
    m_out_msg_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker... STOPPED" << BASH_MSG_RESET);
  ros::shutdown();
}

void hiros::track::SkeletonTracker::setupRosTopics()
{
  for (const auto& topic : m_params.in_skeleton_group_topics) {
    m_in_skeleton_group_subs.push_back(m_nh.subscribe(topic, 1, &SkeletonTracker::detectorCallback, this));
  }

  for (const auto& sub : m_in_skeleton_group_subs) {
    while (sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
      ROS_WARN_STREAM_DELAYED_THROTTLE(2,
                                       "Hi-ROS Skeleton Tracker Warning: No input messages on skeleton group topic(s)");
    }
  }

  m_out_msg_pub = m_nh.advertise<hiros_skeleton_msgs::SkeletonGroup>(m_params.out_msg_topic_name, 1);
}

void hiros::track::SkeletonTracker::detectorCallback(const hiros_skeleton_msgs::SkeletonGroup& t_skeleton_group_msg)
{
  if (!ros::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  if (t_skeleton_group_msg.skeletons.empty() || utils::newestSrcTime(t_skeleton_group_msg) <= getPreviousSrcTime()) {
    return;
  }

  checkFrameIdConsistency(t_skeleton_group_msg);
  addNewSkeletonGroupToBuffer(t_skeleton_group_msg);

  while (!m_skeleton_group_buffer.empty()
         && (utils::oldestSrcTime(t_skeleton_group_msg) - m_skeleton_group_buffer.get_src_time())
              >= m_params.fixed_delay) {
    trackOldestFrame();
    publishTracks();
  }
}

void hiros::track::SkeletonTracker::checkFrameIdConsistency(
  const hiros_skeleton_msgs::SkeletonGroup& t_skeleton_group_msg)
{
  if (m_received_frames.size() < m_n_detectors) {
    m_received_frames[t_skeleton_group_msg.skeletons.front().src_frame] = t_skeleton_group_msg.header.frame_id;
    m_tracks.frame = t_skeleton_group_msg.header.frame_id;

    if (std::find_if(
          m_received_frames.begin(), m_received_frames.end(), [&](const auto& e) { return e.second != m_tracks.frame; })
        != m_received_frames.end()) {
      ROS_FATAL_STREAM("Hi-ROS Skeleton Tracker Error: Data must be expressed w.r.t the same reference frame");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
  }
}

void hiros::track::SkeletonTracker::addNewSkeletonGroupToBuffer(
  const hiros_skeleton_msgs::SkeletonGroup& t_skeleton_group_msg)
{
  m_skeleton_group_buffer.push_back(filterByConfidence(t_skeleton_group_msg));
}

void hiros::track::SkeletonTracker::trackOldestFrame()
{
  fillDetections();
  createCostMatrix();
  solveMunkres();
  removeDistantMatches();
  updateDetectedTracks();
  addNewTracks();
  removeUnassociatedTracks();
}

void hiros::track::SkeletonTracker::publishTracks()
{
  m_tracks.time = ros::Time::now().toSec();
  m_out_msg_pub.publish(skeletons::utils::toMsg(m_tracks));
}

ros::Time hiros::track::SkeletonTracker::getPreviousSrcTime() const
{
  // Return the source time of the latest available tracks
  return utils::newestSrcTime(m_tracks);
}

ros::Time hiros::track::SkeletonTracker::getCurrentSrcTime() const
{
  // If m_detections is already filled, then the current src_time is the max among the detections' src_times
  if (!m_detections.skeletons.empty()) {
    return utils::newestSrcTime(m_detections);
  }
  // Else, if the skeleton_group buffer is not empty, the current src_time is the one from the oldest frame in the
  // buffer
  else if (!m_skeleton_group_buffer.empty()) {
    return m_skeleton_group_buffer.get_src_time();
  }

  return ros::Time();
}

hiros_skeleton_msgs::SkeletonGroup
hiros::track::SkeletonTracker::filterByConfidence(const hiros_skeleton_msgs::SkeletonGroup& t_skeleton_group_msg) const
{
  hiros_skeleton_msgs::SkeletonGroup filtered_skeleton_group = t_skeleton_group_msg;

  // Remove skeletons with confidence < min_skeleton_confidence
  filtered_skeleton_group.skeletons.erase(
    std::remove_if(
      filtered_skeleton_group.skeletons.begin(),
      filtered_skeleton_group.skeletons.end(),
      [&](auto& skel) { return (skel.confidence >= 0 && skel.confidence < m_params.min_skeleton_confidence); }),
    filtered_skeleton_group.skeletons.end());

  // Remove markers with confidence < min_marker_confidence
  for (auto& skel : filtered_skeleton_group.skeletons) {
    skel.markers.erase(std::remove_if(skel.markers.begin(),
                                      skel.markers.end(),
                                      [&](const auto& mk) {
                                        return (mk.confidence >= 0 && mk.confidence < m_params.min_marker_confidence);
                                      }),
                       skel.markers.end());
  }

  // Remove links with confidence < min_link_confidence
  for (auto& skel : filtered_skeleton_group.skeletons) {
    skel.links.erase(std::remove_if(skel.links.begin(),
                                    skel.links.end(),
                                    [&](const auto& lk) {
                                      return (lk.confidence >= 0 && lk.confidence < m_params.min_link_confidence);
                                    }),
                     skel.links.end());
  }

  return filtered_skeleton_group;
}

void hiros::track::SkeletonTracker::fillDetections()
{
  m_detections.skeletons.clear();

  for (const auto& skeleton : m_skeleton_group_buffer.get_skeleton_group().skeletons) {
    if (!utils::isEmpty(skeleton)) {
      m_detections.addSkeleton(hiros::skeletons::utils::toStruct(skeleton));
    }
  }

  eraseOldSkeletonGroupFromBuffer();
}

void hiros::track::SkeletonTracker::createCostMatrix()
{
  if (m_tracks.skeletons.empty() || m_detections.skeletons.empty()) {
    m_cost_matrix = cv::Mat_<double>();
    m_linear_distance_matrix = cv::Mat_<double>();
    m_angular_distance_matrix = cv::Mat_<double>();
    return;
  }

  m_cost_matrix =
    cv::Mat_<double>(static_cast<int>(m_tracks.skeletons.size()), static_cast<int>(m_detections.skeletons.size()));
  m_linear_distance_matrix =
    cv::Mat_<double>(static_cast<int>(m_tracks.skeletons.size()), static_cast<int>(m_detections.skeletons.size()));
  m_angular_distance_matrix =
    cv::Mat_<double>(static_cast<int>(m_tracks.skeletons.size()), static_cast<int>(m_detections.skeletons.size()));

  for (int track_idx = 0; track_idx < m_cost_matrix.rows; ++track_idx) {
    for (int det_idx = 0; det_idx < m_cost_matrix.cols; ++det_idx) {
      auto predicted_track = utils::predict(m_tracks.skeletons.at(static_cast<unsigned int>(track_idx)),
                                            m_detections.skeletons.at(static_cast<unsigned int>(det_idx)).src_time);

      m_linear_distance_matrix(track_idx, det_idx) =
        computeLinearDistance(predicted_track, m_detections.skeletons.at(static_cast<unsigned int>(det_idx)));
      m_angular_distance_matrix(track_idx, det_idx) =
        computeAngularDistance(predicted_track, m_detections.skeletons.at(static_cast<unsigned int>(det_idx)));
      m_cost_matrix(track_idx, det_idx) =
        computeWeightedDistance(predicted_track, m_detections.skeletons.at(static_cast<unsigned int>(det_idx)));
    }
  }
}

void hiros::track::SkeletonTracker::solveMunkres()
{
  // Munkres: rows = tracks, cols = detections
  m_munkres_matrix = m_munkres.solve(m_cost_matrix);
}

void hiros::track::SkeletonTracker::removeDistantMatches()
{
  for (int r = 0; r < m_munkres_matrix.rows; ++r) {
    for (int c = 0; c < m_munkres_matrix.cols; ++c) {
      if (m_munkres_matrix(r, c) == 1
          && (m_linear_distance_matrix[r][c] > m_params.max_linear_distance
              || m_angular_distance_matrix[r][c] > m_params.max_angular_distance)) {
        m_munkres_matrix(r, c) = 0;
      }
    }
  }
}

void hiros::track::SkeletonTracker::updateDetectedTracks()
{
  for (unsigned int track_idx = 0; track_idx < m_tracks.skeletons.size(); ++track_idx) {
    for (unsigned int det_idx = 0; det_idx < m_detections.skeletons.size(); ++det_idx) {
      updateDetectedTrack(track_idx, det_idx);
    }
  }
}

void hiros::track::SkeletonTracker::addNewTracks()
{
  if (m_detections.skeletons.empty()) {
    return;
  }

  if (m_cost_matrix.empty()) {
    for (const auto& detection : m_detections.skeletons) {
      addNewTrack(detection);
    }
  }
  else {
    for (unsigned int c = 0; c < static_cast<unsigned int>(m_munkres_matrix.cols); ++c) {
      if (unassociatedDetection(c)) {
        double min;
        unsigned int row, col;
        utils::minWithIndex(m_cost_matrix.col(static_cast<int>(c)), min, row, col);

        // If the distance between the unassociated detection and the track is lower than params.min_distance,
        // then consider the detection as part of the track
        if (utils::min(m_linear_distance_matrix.col(static_cast<int>(c))) < m_params.min_linear_distance
            && utils::min(m_angular_distance_matrix.col(static_cast<int>(c))) < m_params.min_angular_distance) {
          utils::merge(m_tracks.skeletons.at(row), m_detections.skeletons.at(c));
        }
        else {
          addNewTrack(m_detections.skeletons.at(c));
        }
      }
    }
  }
}

void hiros::track::SkeletonTracker::removeUnassociatedTracks()
{
  unsigned int track_idx;
  int id;
  ros::Duration delta_t;

  for (int i = static_cast<int>(m_tracks.skeletons.size()) - 1; i >= 0; --i) {
    track_idx = static_cast<unsigned int>(i);

    if (unassociatedTrack(track_idx)) {
      id = m_tracks.skeletons.at(track_idx).id;
      delta_t = getCurrentSrcTime() - ros::Time(m_tracks.skeletons.at(track_idx).src_time);

      if (delta_t > m_params.max_delta_t) {
        m_tracks.removeSkeleton(id);
        m_track_filters.erase(id);
      }
    }
  }
}

void hiros::track::SkeletonTracker::eraseOldSkeletonGroupFromBuffer()
{
  m_skeleton_group_buffer.pop_front();
}

double hiros::track::SkeletonTracker::computeWeight(const double& t_det_confidence,
                                                    const hiros::skeletons::types::Vector3& t_track_vel) const
{
  auto weight = (m_params.weight_distances_by_confidences && !std::isnan(t_det_confidence) && t_det_confidence >= 0)
                  ? t_det_confidence
                  : 1;
  if (m_params.weight_distances_by_velocities) {
    // The higher the velocity the lower the marker/link will be weighted. Raise velocity magnitude to the power of
    // -0.25 to have smoother weights: vel = 0 -> weight = 1, vel = 10 -> weight = 0.55, vel = 100 -> weight =
    // 0.32, vel = inf -> weight = 0
    auto vel_weight = std::pow(1 + hiros::skeletons::utils::magnitude(t_track_vel), -0.25);
    weight *= !std::isnan(vel_weight) ? vel_weight : 1;
  }

  return weight;
}

double hiros::track::SkeletonTracker::computeLinPosDistance(const hiros::skeletons::types::KinematicState& t_det_ks,
                                                            const hiros::skeletons::types::KinematicState& t_track_ks,
                                                            const double& t_weight) const
{
  return m_params.use_positions
           ? t_weight * hiros::skeletons::utils::distance(t_det_ks.pose.position, t_track_ks.pose.position)
           : std::numeric_limits<double>::quiet_NaN();
}

double hiros::track::SkeletonTracker::computeLinVelDistance(const hiros::skeletons::types::KinematicState& t_det_ks,
                                                            const hiros::skeletons::types::KinematicState& t_track_ks,
                                                            const double& t_weight) const
{
  return m_params.use_linear_velocities
           ? t_weight * hiros::skeletons::utils::distance(t_det_ks.velocity.linear, t_track_ks.velocity.linear)
           : std::numeric_limits<double>::quiet_NaN();
}

double hiros::track::SkeletonTracker::computeAngPosDistance(const hiros::skeletons::types::KinematicState& t_det_ks,
                                                            const hiros::skeletons::types::KinematicState& t_track_ks,
                                                            const double& t_weight) const
{
  return m_params.use_orientations
           ? t_weight * hiros::skeletons::utils::distance(t_det_ks.pose.orientation, t_track_ks.pose.orientation)
           : std::numeric_limits<double>::quiet_NaN();
}

double hiros::track::SkeletonTracker::computeAngVelDistance(const hiros::skeletons::types::KinematicState& t_det_ks,
                                                            const hiros::skeletons::types::KinematicState& t_track_ks,
                                                            const double& t_weight) const
{
  return m_params.use_angular_velocities
           ? t_weight * hiros::skeletons::utils::distance(t_det_ks.velocity.angular, t_track_ks.velocity.angular)
           : std::numeric_limits<double>::quiet_NaN();
}

double hiros::track::SkeletonTracker::computeMeanDistance(const std::vector<double>& t_pos_dist,
                                                          const std::vector<double>& t_vel_dist,
                                                          const bool& t_weighted) const
{
  auto n_pos = std::count_if(t_pos_dist.begin(), t_pos_dist.end(), [](const auto& e) { return !std::isnan(e); });
  auto n_vel = std::count_if(t_vel_dist.begin(), t_vel_dist.end(), [](const auto& e) { return !std::isnan(e); });

  double pos_dist = std::numeric_limits<double>::quiet_NaN();
  double vel_dist = std::numeric_limits<double>::quiet_NaN();

  auto power = t_weighted ? 1.25 : 1;

  // Divide the average distance by n^0.25 to prefer track-detection matches that have an higher number of
  // markers/links in common
  if (n_pos > 0) {
    pos_dist = std::accumulate(t_pos_dist.begin(), t_pos_dist.end(), 0.0, [&](double a, double b) {
      return a + (std::isnan(b) ? 0. : b / std::pow(n_pos, power));
    });

    if (std::isnan(vel_dist)) {
      vel_dist = 0;
    }
  }

  if (n_vel > 0) {
    vel_dist = std::accumulate(t_pos_dist.begin(), t_pos_dist.end(), 0.0, [&](double a, double b) {
      return a + (std::isnan(b) ? 0. : b / std::pow(n_pos, power));
    });

    if (std::isnan(pos_dist)) {
      vel_dist = 0;
    }
  }

  return pos_dist + m_params.velocity_weight * vel_dist;
}

double hiros::track::SkeletonTracker::computeMarkersLinDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                                const hiros::skeletons::types::Skeleton& t_detection,
                                                                const bool& t_weighted) const
{
  // If positions and linear velocities do not have to be used, then the linear distance is 0
  if (!m_params.use_positions && !m_params.use_linear_velocities) {
    return 0;
  }

  std::vector<double> pos_dist;
  std::vector<double> vel_dist;

  for (const auto& det_mk : t_detection.markers) {
    if (t_track.hasMarker(det_mk.id)) {
      auto& track_mk = t_track.getMarker(det_mk.id);

      auto weight = t_weighted ? computeWeight(det_mk.confidence, track_mk.center.velocity.linear) : 1;
      pos_dist.push_back(computeLinPosDistance(det_mk.center, track_mk.center, weight));
      vel_dist.push_back(computeLinVelDistance(det_mk.center, track_mk.center, weight));
    }
  }

  return computeMeanDistance(pos_dist, vel_dist, t_weighted);
}

double hiros::track::SkeletonTracker::computeMarkersAngDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                                const hiros::skeletons::types::Skeleton& t_detection,
                                                                const bool& t_weighted) const
{
  // If orientations and angular velocities do not have to be used, then the angular distance is 0
  if (!m_params.use_orientations && !m_params.use_angular_velocities) {
    return 0;
  }

  std::vector<double> or_dist;
  std::vector<double> vel_dist;

  for (const auto& det_mk : t_detection.markers) {
    if (t_track.hasMarker(det_mk.id)) {
      auto& track_mk = t_track.getMarker(det_mk.id);

      auto weight = t_weighted ? computeWeight(det_mk.confidence, track_mk.center.velocity.angular) : 1;
      or_dist.push_back(computeAngPosDistance(det_mk.center, track_mk.center, weight));
      vel_dist.push_back(computeAngVelDistance(det_mk.center, track_mk.center, weight));
    }
  }

  return computeMeanDistance(or_dist, vel_dist, t_weighted);
}

double hiros::track::SkeletonTracker::computeLinksLinDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                              const hiros::skeletons::types::Skeleton& t_detection,
                                                              const bool& t_weighted) const
{
  // If positions and linear velocities do not have to be used, then the linear distance is 0
  if (!m_params.use_positions && !m_params.use_linear_velocities) {
    return 0;
  }

  std::vector<double> pos_dist;
  std::vector<double> vel_dist;

  for (const auto& det_lk : t_detection.links) {
    if (t_track.hasLink(det_lk.id)) {
      auto& track_lk = t_track.getLink(det_lk.id);

      auto weight = t_weighted ? computeWeight(det_lk.confidence, track_lk.center.velocity.linear) : 1;
      pos_dist.push_back(computeLinPosDistance(det_lk.center, track_lk.center, weight));
      vel_dist.push_back(computeLinVelDistance(det_lk.center, track_lk.center, weight));
    }
  }

  return computeMeanDistance(pos_dist, vel_dist, t_weighted);
}

double hiros::track::SkeletonTracker::computeLinksAngDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                              const hiros::skeletons::types::Skeleton& t_detection,
                                                              const bool& t_weighted) const
{
  // If orientations and angular velocities do not have to be used, then the angular distance is 0
  if (!m_params.use_orientations && !m_params.use_angular_velocities) {
    return 0;
  }

  std::vector<double> or_dist;
  std::vector<double> vel_dist;

  for (const auto& det_lk : t_detection.links) {
    if (t_track.hasLink(det_lk.id)) {
      auto& track_lk = t_track.getLink(det_lk.id);

      auto weight = t_weighted ? computeWeight(det_lk.confidence, track_lk.center.velocity.angular) : 1;
      or_dist.push_back(computeAngPosDistance(det_lk.center, track_lk.center, weight));
      vel_dist.push_back(computeAngVelDistance(det_lk.center, track_lk.center, weight));
    }
  }

  return computeMeanDistance(or_dist, vel_dist, t_weighted);
}

double hiros::track::SkeletonTracker::computeLinearDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                            const hiros::skeletons::types::Skeleton& t_detection,
                                                            const bool& t_weighted) const
{
  // If positions and linear velocities do not have to be used, then the linear distance is 0
  if (!m_params.use_positions && !m_params.use_linear_velocities) {
    return 0;
  }

  auto marker_dist = computeMarkersLinDistance(t_track, t_detection, t_weighted);
  auto link_dist = computeLinksLinDistance(t_track, t_detection, t_weighted);

  if (std::isnan(marker_dist) && std::isnan(link_dist)) {
    return std::numeric_limits<double>::max();
  }

  if (std::isnan(marker_dist)) {
    marker_dist = 0;
  }

  if (std::isnan(link_dist)) {
    link_dist = 0;
  }

  return (marker_dist + link_dist) / 2;
}

double hiros::track::SkeletonTracker::computeAngularDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                             const hiros::skeletons::types::Skeleton& t_detection,
                                                             const bool& t_weighted) const
{
  // If orientations and angular velocities do not have to be used, then the angular distance is 0
  if (!m_params.use_orientations && !m_params.use_angular_velocities) {
    return 0;
  }

  auto marker_dist = computeMarkersAngDistance(t_track, t_detection, t_weighted);
  auto link_dist = computeLinksAngDistance(t_track, t_detection, t_weighted);

  if (std::isnan(marker_dist) && std::isnan(link_dist)) {
    return std::numeric_limits<double>::max();
  }

  if (std::isnan(marker_dist)) {
    marker_dist = 0;
  }

  if (std::isnan(link_dist)) {
    link_dist = 0;
  }

  return (marker_dist + link_dist) / 2;
}

double
hiros::track::SkeletonTracker::computeWeightedDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                       const hiros::skeletons::types::Skeleton& t_detection) const
{
  auto filtered_det = t_detection;
  m_track_filters.at(t_track.id).updVelAndAcc(filtered_det, k_cutoff_frequency);

  return computeLinearDistance(t_track, filtered_det, true) + computeAngularDistance(t_track, filtered_det, true);
}

void hiros::track::SkeletonTracker::initializeVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton) const
{
  for (auto& mk : t_skeleton.markers) {
    initializeVelAndAcc(mk.center);
  }

  for (auto& lk : t_skeleton.links) {
    initializeVelAndAcc(lk.center);
  }
}

void hiros::track::SkeletonTracker::initializeVelAndAcc(hiros::skeletons::types::KinematicState& t_state) const
{
  if (hiros::skeletons::utils::isNaN(t_state.velocity.linear)) {
    t_state.velocity.linear = hiros::skeletons::types::Vector3(0, 0, 0);
  }
  if (hiros::skeletons::utils::isNaN(t_state.velocity.angular)) {
    t_state.velocity.angular = hiros::skeletons::types::Vector3(0, 0, 0);
  }

  if (hiros::skeletons::utils::isNaN(t_state.acceleration.linear)) {
    t_state.acceleration.linear = hiros::skeletons::types::Vector3(0, 0, 0);
  }
  if (hiros::skeletons::utils::isNaN(t_state.acceleration.angular)) {
    t_state.acceleration.angular = hiros::skeletons::types::Vector3(0, 0, 0);
  }
}

void hiros::track::SkeletonTracker::updateDetectedTrack(const unsigned int& t_track_idx, const unsigned int& t_det_idx)
{
  if (utils::matchMunkres(m_munkres_matrix, t_track_idx, t_det_idx)) {
    int id = m_tracks.skeletons.at(t_track_idx).id;
    m_tracks.skeletons.at(t_track_idx) = m_detections.skeletons.at(t_det_idx);
    m_tracks.skeletons.at(t_track_idx).id = id;
    m_track_filters[id].updVelAndAcc(m_tracks.skeletons.at(t_track_idx), k_cutoff_frequency);
  }
}

void hiros::track::SkeletonTracker::addNewTrack(const hiros::skeletons::types::Skeleton& t_detection)
{
  if (t_detection.markers.size() >= static_cast<unsigned int>(m_params.min_markers)
      && t_detection.links.size() >= static_cast<unsigned int>(m_params.min_links)) {
    m_tracks.addSkeleton(t_detection);
    auto& new_track = m_tracks.getSkeleton(t_detection.id);
    new_track.id = ++m_last_track_id;
    initializeVelAndAcc(new_track);
    m_track_filters[new_track.id] = Filter(new_track, k_cutoff_frequency);
  }
}

bool hiros::track::SkeletonTracker::unassociatedDetection(const unsigned int& t_det_idx) const
{
  return !m_munkres.colHasMatch(t_det_idx);
}

bool hiros::track::SkeletonTracker::unassociatedTrack(const unsigned int& t_track_idx) const
{
  return !m_munkres.rowHasMatch(t_track_idx);
}
