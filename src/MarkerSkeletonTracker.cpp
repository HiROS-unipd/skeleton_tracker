// ROS dependencies
#include <ros/ros.h>
#include <std_msgs/Header.h>

// Custom External Packages dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/MarkerSkeletonTracker.h"
#include "skeleton_tracker/utils.h"

hiros::track::MarkerSkeletonTracker::MarkerSkeletonTracker()
  : m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_last_track_id(-1)
  , m_configured(false)
{}

hiros::track::MarkerSkeletonTracker::~MarkerSkeletonTracker() {}

void hiros::track::MarkerSkeletonTracker::configure()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker...Configuring");

  if (m_configured) {
    m_configured = false;
    stop();
  }

  m_nh.getParam("node_name", m_params.node_name);
  m_nh.getParam("in_skeleton_group_topics", m_params.in_skeleton_group_topics);
  m_nh.getParam("out_msg_topic_name", m_params.out_msg_topic_name);

  if (m_params.in_skeleton_group_topics.empty() || m_params.out_msg_topic_name.empty()) {
    ROS_FATAL_STREAM("Required topics configuration not provided. Unable to continue");
    ros::shutdown();
  }

  m_n_detectors = m_params.in_skeleton_group_topics.size();
  double max_delta_t = 0;
  m_nh.getParam("fixed_delay", m_params.fixed_delay);
  m_nh.getParam("min_markers", m_params.min_markers);
  m_nh.getParam("min_distance", m_params.min_distance);
  m_nh.getParam("max_distance", m_params.max_distance);
  m_nh.getParam("max_delta_t", max_delta_t);
  m_params.max_delta_t = ros::Duration(max_delta_t);
  m_nh.getParam("use_marker_positions", m_params.use_marker_positions);
  m_nh.getParam("use_marker_velocities", m_params.use_marker_velocities);
  m_nh.getParam("velocity_weight", m_params.velocity_weight);
  m_nh.getParam("weight_distances_by_confidences", m_params.weight_distances_by_confidences);
  m_nh.getParam("weight_distances_by_velocities", m_params.weight_distances_by_velocities);

  m_nh.getParam("filter_marker_trajectories", m_params.filter_marker_trajectories);
  m_nh.getParam("filter_cutoff_frequency", m_params.filter_cutoff_frequency);

  if (m_params.filter_marker_trajectories && (m_params.filter_cutoff_frequency <= 0.0)) {
    ROS_FATAL_STREAM("Cutoff frequency must be greater than 0 Hz. Unable to continue");
    ros::shutdown();
  }

  if (!m_params.use_marker_positions && !m_params.use_marker_velocities) {
    ROS_FATAL_STREAM("Position based distance and/or velocity based distance must be enabled. Unable to continue");
    ros::shutdown();
  }

  m_configured = true;
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker...CONFIGURED" << BASH_MSG_RESET);
}

void hiros::track::MarkerSkeletonTracker::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker...Starting");

  if (!m_configured) {
    configure();
  }

  setupRosTopics();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker...RUNNING" << BASH_MSG_RESET);
}

void hiros::track::MarkerSkeletonTracker::stop()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker...Stopping");

  if (!m_in_skeleton_group_subs.empty()) {
    for (auto& sub : m_in_skeleton_group_subs)
      sub.shutdown();
  }

  if (m_out_msg_pub) {
    m_out_msg_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker...STOPPED" << BASH_MSG_RESET);
}

void hiros::track::MarkerSkeletonTracker::setupRosTopics()
{
  for (const auto& topic : m_params.in_skeleton_group_topics) {
    m_in_skeleton_group_subs.push_back(m_nh.subscribe(topic, 1, &MarkerSkeletonTracker::detectorCallback, this));
  }

  for (const auto& sub : m_in_skeleton_group_subs) {
    while (sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
      ROS_WARN_STREAM_THROTTLE(2, m_node_namespace << " No input messages on skeleton group topic(s)");
    }
  }

  m_out_msg_pub = m_nh.advertise<hiros_skeleton_msgs::MarkerSkeletonGroup>(m_params.out_msg_topic_name, 1);
}

void hiros::track::MarkerSkeletonTracker::checkFrameIdConsistency(
  hiros_skeleton_msgs::MarkerSkeletonGroupConstPtr t_skeleton_group_msg)
{
  if (m_received_frames.size() < m_n_detectors) {
    auto src_frame = t_skeleton_group_msg->src_frame;

    if (std::find_if(m_received_frames.begin(),
                     m_received_frames.end(),
                     [&src_frame](const auto& elem) { return elem.second == src_frame; })
        == m_received_frames.end()) {
      m_received_frames.push_back(
        std::make_pair(t_skeleton_group_msg->src_frame, t_skeleton_group_msg->header.frame_id));
    }

    if (m_received_frames.size() == m_n_detectors) {
      m_frame_id = m_received_frames.front().second;

      for (const auto& frame_pair : m_received_frames) {
        if (frame_pair.second != m_frame_id) {
          ROS_FATAL_STREAM("Error. Data must be expressed w.r.t the same reference frame");
          ros::shutdown();
          exit(EXIT_FAILURE);
        }
      }
    }
  }
}

void hiros::track::MarkerSkeletonTracker::detectorCallback(
  hiros_skeleton_msgs::MarkerSkeletonGroupConstPtr t_skeleton_group_msg)
{
  checkFrameIdConsistency(t_skeleton_group_msg);

  if (t_skeleton_group_msg->src_time <= getPreviousSourceTime()) {
    return;
  }

  addNewSkeletonGroupToBuffer(t_skeleton_group_msg);

  if (m_n_detectors == 1) {
    trackOldestFrame();
    publishTracks(m_tracks);
  }
  else {
    bool ready_to_publish = false;

    while ((t_skeleton_group_msg->src_time.toSec() - m_skeleton_groups_buffer.begin()->second.src_time)
             >= m_params.fixed_delay
           && !m_skeleton_groups_buffer.empty()) {
      trackOldestFrame();
      mergeTracks(ready_to_publish);
      if (ready_to_publish) {
        publishTracks(m_avg_tracks);
      }
    }
  }
}

void hiros::track::MarkerSkeletonTracker::trackOldestFrame()
{
  m_tracks.src_time = m_skeleton_groups_buffer.begin()->second.src_time;
  m_tracks.src_frame = m_skeleton_groups_buffer.begin()->second.src_frame;

  fillDetections();
  createCostMatrix();
  solveMunkres();
  removeDistantMatches();
  updateDetectedTracks();
  addNewTracks();
  removeUnassociatedTracks();
  eraseOldSkeletonGroupFromBuffer();
}

ros::Time hiros::track::MarkerSkeletonTracker::getPreviousSourceTime() const
{
  return !m_track_id_to_time_stamp_map.empty()
           ? (std::max_element(m_track_id_to_time_stamp_map.begin(),
                               m_track_id_to_time_stamp_map.end(),
                               [](const std::pair<int, ros::Time>& t1, const std::pair<int, ros::Time>& t2) {
                                 return t1.second < t2.second;
                               }))
               ->second
           : ros::Time();
}

void hiros::track::MarkerSkeletonTracker::addNewSkeletonGroupToBuffer(
  hiros_skeleton_msgs::MarkerSkeletonGroupConstPtr t_skeleton_group_msg)
{
  m_skeleton_groups_buffer.emplace(t_skeleton_group_msg->src_time,
                                   hiros::skeletons::utils::toStruct(*t_skeleton_group_msg));
}

void hiros::track::MarkerSkeletonTracker::eraseOldSkeletonGroupFromBuffer()
{
  m_skeleton_groups_buffer.erase(m_skeleton_groups_buffer.begin());
}

void hiros::track::MarkerSkeletonTracker::mergeTracks(bool& t_ready_to_be_published)
{
  t_ready_to_be_published = false;

  auto last_src_frame = m_tracks.src_frame;

  if (std::find_if(
        m_frames_to_merge.begin(),
        m_frames_to_merge.end(),
        [&last_src_frame](const std::pair<ros::Time, std::string>& elem) { return elem.second == last_src_frame; })
      != m_frames_to_merge.end()) {
    // Add previous tracks to the computation of the average tracks when some frames are missing
    for (unsigned long i = m_frames_to_merge.size(); i < m_n_detectors; ++i) {
      for (const auto& track : m_avg_tracks.marker_skeletons) {
        m_tracks_to_merge[track.id].push_back(track);
      }
    }

    computeAvgTracks();

    m_frames_to_merge.clear();
    m_tracks_to_merge.clear();

    t_ready_to_be_published = true;
  }

  m_frames_to_merge.push_back(std::make_pair(ros::Time(m_tracks.src_time), m_tracks.src_frame));

  for (const auto& skeleton : m_tracks.marker_skeletons) {
    m_tracks_to_merge[skeleton.id].push_back(skeleton);
  }

  if (m_frames_to_merge.size() == m_n_detectors) {
    computeAvgTracks();

    m_frames_to_merge.clear();
    m_tracks_to_merge.clear();

    t_ready_to_be_published = true;
  }
}

void hiros::track::MarkerSkeletonTracker::publishTracks(
  const hiros::skeletons::types::MarkerSkeletonGroup& t_tracks) const
{
  m_out_msg_pub.publish(
    skeletons::utils::toMsg(ros::Time::now(), m_frame_id, ros::Time(t_tracks.src_time), t_tracks.src_frame, t_tracks));
}

void hiros::track::MarkerSkeletonTracker::fillDetections()
{
  m_detections.marker_skeletons.clear();

  m_detections.src_time = m_skeleton_groups_buffer.begin()->second.src_time;
  m_detections.src_frame = m_skeleton_groups_buffer.begin()->second.src_frame;
  for (const auto& skeleton : m_skeleton_groups_buffer.begin()->second.marker_skeletons) {
    if (!utils::isEmpty(skeleton)) {
      m_detections.addMarkerSkeleton(skeleton);
    }
  }
}

void hiros::track::MarkerSkeletonTracker::createCostMatrix()
{
  if (m_tracks.marker_skeletons.empty() || m_detections.marker_skeletons.empty()) {
    m_cost_matrix = cv::Mat_<double>();
    return;
  }

  auto tracks_ptr = m_avg_tracks.marker_skeletons.empty()
                      ? std::make_unique<hiros::skeletons::types::MarkerSkeletonGroup>(m_tracks)
                      : std::make_unique<hiros::skeletons::types::MarkerSkeletonGroup>(m_avg_tracks);

  m_cost_matrix = cv::Mat_<double>(static_cast<int>(tracks_ptr->marker_skeletons.size()),
                                   static_cast<int>(m_detections.marker_skeletons.size()));

  for (int track_idx = 0; track_idx < m_cost_matrix.rows; ++track_idx) {
    for (int det_idx = 0; det_idx < m_cost_matrix.cols; ++det_idx) {
      m_cost_matrix(track_idx, det_idx) =
        computeDistance(tracks_ptr->marker_skeletons.at(static_cast<unsigned int>(track_idx)),
                        m_detections.marker_skeletons.at(static_cast<unsigned int>(det_idx)));
    }
  }

  utils::replaceNans(m_cost_matrix);
}

void hiros::track::MarkerSkeletonTracker::solveMunkres()
{
  // Munkres: rows = tracks, cols = detections
  m_munkres_matrix = m_munkres.solve(m_cost_matrix);
}

void hiros::track::MarkerSkeletonTracker::removeDistantMatches()
{
  if (m_params.max_distance >= 0) {
    for (int r = 0; r < m_munkres_matrix.rows; ++r) {
      for (int c = 0; c < m_munkres_matrix.cols; ++c) {
        if (m_munkres_matrix(r, c) == 1 && m_cost_matrix(r, c) > m_params.max_distance) {
          m_munkres_matrix(r, c) = 0;
        }
      }
    }
  }
}

void hiros::track::MarkerSkeletonTracker::updateDetectedTracks()
{
  for (unsigned int track_idx = 0; track_idx < m_tracks.marker_skeletons.size(); ++track_idx) {
    for (unsigned int det_idx = 0; det_idx < m_detections.marker_skeletons.size(); ++det_idx) {
      updateDetectedTrack(track_idx, det_idx);
    }
  }
}

void hiros::track::MarkerSkeletonTracker::addNewTracks()
{
  if (m_detections.marker_skeletons.empty()) {
    return;
  }

  if (m_cost_matrix.empty()) {
    for (const auto& detection : m_detections.marker_skeletons) {
      addNewTrack(detection);
    }
  }
  else {
    for (unsigned int c = 0; c < static_cast<unsigned int>(m_munkres_matrix.cols); ++c) {
      if (unassociatedDetection(c) && m_params.min_distance >= 0) {

        double min;
        unsigned int row, col;
        utils::minWithIndex(m_cost_matrix.col(static_cast<int>(c)), min, row, col);

        if (min < m_params.min_distance) {
          utils::merge(m_tracks.marker_skeletons.at(row), m_detections.marker_skeletons.at(c));
        }
        else {
          addNewTrack(m_detections.marker_skeletons.at(c));
        }
      }
    }
  }
}

void hiros::track::MarkerSkeletonTracker::removeUnassociatedTracks()
{
  int id;
  ros::Duration delta_t;

  for (unsigned int track_idx = 0, idx_to_erase = 0; track_idx < m_tracks.marker_skeletons.size();
       ++track_idx, ++idx_to_erase) {
    if (unassociatedTrack(track_idx)) {
      id = m_tracks.marker_skeletons.at(idx_to_erase).id;
      delta_t = m_skeleton_groups_buffer.begin()->first - m_track_id_to_time_stamp_map.at(id);

      if (delta_t > m_params.max_delta_t) {
        if (m_n_detectors == 1 && m_params.filter_marker_trajectories) {
          m_track_id_to_filter_map.erase(id);
        }

        m_tracks_to_merge.erase(id);
        m_track_id_to_time_stamp_map.erase(id);
        m_tracks.marker_skeletons.erase(m_tracks.marker_skeletons.begin() + idx_to_erase--);
      }
    }
  }
}

double hiros::track::MarkerSkeletonTracker::computeDistance(const hiros::skeletons::types::MarkerSkeleton& t_track,
                                                            hiros::skeletons::types::MarkerSkeleton& t_detection) const
{
  double pos_dist = 0;
  double vel_dist = 0;
  unsigned int pos_n_mks = 0;
  unsigned int vel_n_mks = 0;
  double weight;

  if (m_params.use_marker_velocities || m_params.weight_distances_by_velocities) {
    computeVelocities(t_track,
                      t_detection,
                      (m_skeleton_groups_buffer.begin()->first - m_track_id_to_time_stamp_map.at(t_track.id)).toSec());
  }

  for (const auto& det_mkg : t_detection.marker_groups) {
    for (const auto& det_mk : det_mkg.second.markers) {
      if (hiros::skeletons::utils::hasMarker(t_track, det_mkg.first, det_mk.first)) {
        weight = (m_params.weight_distances_by_confidences) ? det_mk.second.confidence : 1;

        if (m_params.weight_distances_by_velocities) {
          // Elevate velocity magnitude to the power of 0.25 to have smoother weights
          weight *= std::pow(1
                               + hiros::skeletons::utils::magnitude(
                                 t_track.marker_groups.at(det_mkg.first).markers.at(det_mk.first).point.velocity),
                             -0.25);
        }

        if (m_params.use_marker_positions) {
          pos_dist += (weight
                       * hiros::skeletons::utils::distance(
                         det_mk.second.point.position,
                         t_track.marker_groups.at(det_mkg.first).markers.at(det_mk.first).point.position));
          ++pos_n_mks;
        }

        if (m_params.use_marker_velocities) {
          vel_dist += (weight
                       * hiros::skeletons::utils::distance(
                         det_mk.second.point.velocity,
                         t_track.marker_groups.at(det_mkg.first).markers.at(det_mk.first).point.velocity));
          ++vel_n_mks;
        }
      }
    }
  }

  // Divide the average distance by n_mks^0.25 to prefer track-detection matches that have an higher number of
  // markers in common
  if (pos_n_mks > 0) {
    pos_dist /= std::pow(pos_n_mks, 1.25);
  }
  if (vel_n_mks > 0) {
    vel_dist /= std::pow(vel_n_mks, 1.25);
  }

  return ((pos_n_mks + vel_n_mks) > 0) ? pos_dist + m_params.velocity_weight * vel_dist
                                       : std::numeric_limits<double>::quiet_NaN();
}

void hiros::track::MarkerSkeletonTracker::initializeVelAndAcc(hiros::skeletons::types::MarkerSkeleton& t_skeleton) const
{
  for (auto& mkg : t_skeleton.marker_groups) {
    for (auto& mk : mkg.second.markers) {
      mk.second.point.velocity = hiros::skeletons::types::Velocity();
      mk.second.point.acceleration = hiros::skeletons::types::Acceleration();
    }
  }
}

void hiros::track::MarkerSkeletonTracker::computeVelAndAcc(const hiros::skeletons::types::MarkerSkeleton& t_track,
                                                           hiros::skeletons::types::MarkerSkeleton& t_detection,
                                                           const double& t_dt) const
{
  for (auto& det_mkg : t_detection.marker_groups) {
    for (auto& det_mk : det_mkg.second.markers) {
      if (hiros::skeletons::utils::hasMarker(t_track, det_mkg.first, det_mk.first)) {
        det_mk.second.point.velocity = computeVelocity(
          t_track.marker_groups.at(det_mkg.first).markers.at(det_mk.first).point, det_mk.second.point, t_dt);
        det_mk.second.point.acceleration = computeAcceleration(
          t_track.marker_groups.at(det_mkg.first).markers.at(det_mk.first).point, det_mk.second.point, t_dt);
      }
    }
  }
}

void hiros::track::MarkerSkeletonTracker::computeVelocities(const hiros::skeletons::types::MarkerSkeleton& t_track,
                                                            hiros::skeletons::types::MarkerSkeleton& t_detection,
                                                            const double& t_dt) const
{
  for (auto& det_mkg : t_detection.marker_groups) {
    for (auto& det_mk : det_mkg.second.markers) {
      if (hiros::skeletons::utils::hasMarker(t_track, det_mkg.first, det_mk.first)) {
        det_mk.second.point.velocity = computeVelocity(
          t_track.marker_groups.at(det_mkg.first).markers.at(det_mk.first).point, det_mk.second.point, t_dt);
      }
    }
  }
}

hiros::skeletons::types::Velocity
hiros::track::MarkerSkeletonTracker::computeVelocity(const hiros::skeletons::types::Point& t_prev,
                                                     const hiros::skeletons::types::Point& t_curr,
                                                     const double& t_dt) const
{
  return hiros::skeletons::types::Velocity((t_curr.position.x() - t_prev.position.x()) / t_dt,
                                           (t_curr.position.y() - t_prev.position.y()) / t_dt,
                                           (t_curr.position.z() - t_prev.position.z()) / t_dt);
}

hiros::skeletons::types::Acceleration
hiros::track::MarkerSkeletonTracker::computeAcceleration(const hiros::skeletons::types::Point& t_prev,
                                                         const hiros::skeletons::types::Point& t_curr,
                                                         const double& t_dt) const
{
  return hiros::skeletons::types::Acceleration((t_curr.velocity.x() - t_prev.velocity.x()) / t_dt,
                                               (t_curr.velocity.y() - t_prev.velocity.y()) / t_dt,
                                               (t_curr.velocity.z() - t_prev.velocity.z()) / t_dt);
}

void hiros::track::MarkerSkeletonTracker::updateDetectedTrack(const unsigned int& t_track_idx,
                                                              const unsigned int& t_det_idx)
{
  if (m_munkres.match(t_track_idx, t_det_idx)) {
    int id = m_tracks.marker_skeletons.at(t_track_idx).id;

    if (m_n_detectors == 1 && !m_params.filter_marker_trajectories) {
      computeVelAndAcc(m_tracks.marker_skeletons.at(t_track_idx),
                       m_detections.marker_skeletons.at(t_det_idx),
                       (m_skeleton_groups_buffer.begin()->first - m_track_id_to_time_stamp_map.at(id)).toSec());
    }

    m_track_id_to_time_stamp_map.at(id) = m_skeleton_groups_buffer.begin()->first;

    m_tracks.marker_skeletons.at(t_track_idx) = m_detections.marker_skeletons.at(t_det_idx);
    m_tracks.marker_skeletons.at(t_track_idx).id = id;

    if (m_n_detectors == 1 && m_params.filter_marker_trajectories) {
      m_track_id_to_filter_map.at(id).filter(m_tracks.marker_skeletons.at(t_track_idx),
                                             m_track_id_to_time_stamp_map.at(id).toSec());
    }
  }
}

void hiros::track::MarkerSkeletonTracker::addNewTrack(const hiros::skeletons::types::MarkerSkeleton& t_detection)
{
  if (hiros::skeletons::utils::numberOfMarkers(t_detection) >= m_params.min_markers) {
    m_tracks.addMarkerSkeleton(t_detection);
    m_tracks.marker_skeletons.back().id = ++m_last_track_id;
    initializeVelAndAcc(m_tracks.marker_skeletons.back());

    m_track_id_to_time_stamp_map.emplace(m_last_track_id, m_skeleton_groups_buffer.begin()->first);

    if (m_n_detectors == 1 && m_params.filter_marker_trajectories) {
      m_track_id_to_filter_map.emplace(m_tracks.marker_skeletons.back().id,
                                       hiros::track::Filter(m_tracks.marker_skeletons.back(),
                                                            m_track_id_to_time_stamp_map.at(m_last_track_id).toSec(),
                                                            m_params.filter_cutoff_frequency));
    }
  }
}

bool hiros::track::MarkerSkeletonTracker::unassociatedDetection(const unsigned int& t_det_idx) const
{
  return !m_munkres.colHasMatch(t_det_idx);
}

bool hiros::track::MarkerSkeletonTracker::unassociatedTrack(const unsigned int& t_track_idx) const
{
  return !m_munkres.rowHasMatch(t_track_idx);
}

void hiros::track::MarkerSkeletonTracker::computeAvgTracks()
{
  auto prev_avg_tracks = m_avg_tracks;

  m_avg_tracks.marker_skeletons.clear();

  m_avg_tracks.src_time = computeAvgSrcTime();

  for (const auto& track : m_tracks_to_merge) {
    auto avg_track = computeAvgTrack(track.second);
    auto prev_avg_track = hiros::skeletons::utils::getMarkerSkeleton(prev_avg_tracks, avg_track.id);

    if (m_params.filter_marker_trajectories) {
      if (prev_avg_track == nullptr) {
        // new track
        m_track_id_to_filter_map.emplace(
          avg_track.id, hiros::track::Filter(avg_track, m_avg_tracks.src_time, m_params.filter_cutoff_frequency));
      }
      else {
        m_track_id_to_filter_map.at(avg_track.id).filter(avg_track, m_avg_tracks.src_time);
      }
    }
    else {
      if (prev_avg_track == nullptr) {
        // new track
        initializeVelAndAcc(avg_track);
      }
      else {
        computeVelAndAcc(*prev_avg_track, avg_track, (m_avg_tracks.src_time - prev_avg_tracks.src_time));
      }
    }

    m_avg_tracks.addMarkerSkeleton(avg_track);
  }
}

double hiros::track::MarkerSkeletonTracker::computeAvgSrcTime() const
{
  if (m_frames_to_merge.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double avg_src_time = m_frames_to_merge.front().first.toSec();

  for (unsigned int i = 1; i < m_frames_to_merge.size(); ++i) {
    avg_src_time = (i * avg_src_time + m_frames_to_merge.at(i).first.toSec()) / (i + 1);
  }

  return avg_src_time;
}

hiros::skeletons::types::MarkerSkeleton hiros::track::MarkerSkeletonTracker::computeAvgTrack(
  const std::vector<hiros::skeletons::types::MarkerSkeleton>& t_tracks) const
{
  hiros::skeletons::types::MarkerSkeleton avg_sk = t_tracks.front();

  for (unsigned int i = 1; i < t_tracks.size(); ++i) {
    utils::merge(avg_sk, t_tracks.at(i), i, 1, true);
  }

  return avg_sk;
}
