// ROS dependencies
#include <ros/ros.h>
#include <std_msgs/Header.h>

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
  , m_ok_to_publish(false)
  , m_configured(false)
{}

hiros::track::SkeletonTracker::~SkeletonTracker() {}

void hiros::track::SkeletonTracker::configure()
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
  m_nh.getParam("min_joints", m_params.min_joints);
  m_nh.getParam("min_markers_distance", m_params.min_markers_distance);
  if (m_params.min_markers_distance <= 0) {
    m_params.min_markers_distance = std::numeric_limits<double>::max();
  }
  m_nh.getParam("max_markers_distance", m_params.max_markers_distance);
  if (m_params.max_markers_distance <= 0) {
    m_params.max_markers_distance = std::numeric_limits<double>::max();
  }
  m_nh.getParam("min_orientations_distance", m_params.min_orientations_distance);
  if (m_params.min_orientations_distance <= 0) {
    m_params.min_orientations_distance = std::numeric_limits<double>::max();
  }
  m_nh.getParam("max_orientations_distance", m_params.max_orientations_distance);
  if (m_params.max_orientations_distance <= 0) {
    m_params.max_orientations_distance = std::numeric_limits<double>::max();
  }
  m_nh.getParam("max_delta_t", max_delta_t);
  m_params.max_delta_t = ros::Duration(max_delta_t);
  m_nh.getParam("use_markers", m_params.use_markers);
  m_nh.getParam("use_marker_velocities", m_params.use_marker_velocities);
  m_nh.getParam("use_orientations", m_params.use_orientations);
  m_nh.getParam("use_orientation_velocities", m_params.use_orientation_velocities);
  m_nh.getParam("velocity_weight", m_params.velocity_weight);
  m_nh.getParam("weight_distances_by_confidences", m_params.weight_distances_by_confidences);
  m_nh.getParam("weight_distances_by_velocities", m_params.weight_distances_by_velocities);

  m_nh.getParam("filter_trajectories", m_params.filter_trajectories);
  m_nh.getParam("filter_cutoff_frequency", m_params.filter_cutoff_frequency);

  if (m_params.filter_trajectories && (m_params.filter_cutoff_frequency <= 0.0)) {
    ROS_FATAL_STREAM("Cutoff frequency must be greater than 0 Hz. Unable to continue");
    ros::shutdown();
  }

  if (!m_params.use_markers && !m_params.use_orientations && !m_params.use_marker_velocities
      && !m_params.use_orientation_velocities) {
    ROS_FATAL_STREAM(
      "Position/orientation based distances and/or velocity based distances must be enabled. Unable to continue");
    ros::shutdown();
  }

  setupRosTopics();

  m_configured = true;
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker...CONFIGURED" << BASH_MSG_RESET);
}

void hiros::track::SkeletonTracker::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker...Starting");

  if (!m_configured) {
    configure();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker...RUNNING" << BASH_MSG_RESET);
}

void hiros::track::SkeletonTracker::stop()
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

void hiros::track::SkeletonTracker::setupRosTopics()
{
  for (const auto& topic : m_params.in_skeleton_group_topics) {
    m_in_skeleton_group_subs.push_back(m_nh.subscribe(topic, 1, &SkeletonTracker::detectorCallback, this));
  }

  for (const auto& sub : m_in_skeleton_group_subs) {
    while (sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
      ROS_WARN_STREAM_THROTTLE(2, m_node_namespace << " No input messages on skeleton group topic(s)");
    }
  }

  m_out_msg_pub = m_nh.advertise<hiros_skeleton_msgs::SkeletonGroup>(m_params.out_msg_topic_name, 1);
}

void hiros::track::SkeletonTracker::checkFrameIdConsistency(
  hiros_skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  if (m_received_frames.size() < m_n_detectors) {
    m_received_frames[t_skeleton_group_msg->src_frame] = t_skeleton_group_msg->header.frame_id;
    m_frame_id = t_skeleton_group_msg->header.frame_id;

    if (std::find_if(
          m_received_frames.begin(), m_received_frames.end(), [&](const auto& e) { return e.second != m_frame_id; })
        != m_received_frames.end()) {
      ROS_FATAL_STREAM("Error. Data must be expressed w.r.t the same reference frame");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
  }
}

void hiros::track::SkeletonTracker::detectorCallback(hiros_skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
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
    while ((t_skeleton_group_msg->src_time - m_skeleton_groups_buffer.get_src_time()).toSec() >= m_params.fixed_delay
           && !m_skeleton_groups_buffer.empty()) {
      trackOldestFrame();
      mergeTracks();
      if (m_ok_to_publish) {
        publishTracks(m_avg_tracks);
      }
    }
  }
}

void hiros::track::SkeletonTracker::trackOldestFrame()
{
  m_tracks.src_time = m_skeleton_groups_buffer.get_src_time().toSec();
  m_tracks.src_frame = m_skeleton_groups_buffer.get_src_frame();

  fillDetections();
  createCostMatrix();
  solveMunkres();
  removeDistantMatches();
  updateDetectedTracks();
  addNewTracks();
  removeUnassociatedTracks();
  eraseOldSkeletonGroupFromBuffer();
}

ros::Time hiros::track::SkeletonTracker::getPreviousSourceTime() const
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

void hiros::track::SkeletonTracker::addNewSkeletonGroupToBuffer(
  hiros_skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  m_skeleton_groups_buffer.push_back(t_skeleton_group_msg);
}

void hiros::track::SkeletonTracker::eraseOldSkeletonGroupFromBuffer()
{
  m_skeleton_groups_buffer.pop_back();
}

void hiros::track::SkeletonTracker::mergeTracks()
{
  m_ok_to_publish = false;
  auto last_src_frame = m_tracks.src_frame;

  if (std::find_if(
        m_frames_to_merge.begin(),
        m_frames_to_merge.end(),
        [&last_src_frame](const std::pair<ros::Time, std::string>& elem) { return elem.second == last_src_frame; })
      != m_frames_to_merge.end()) {
    // Add previous tracks to the computation of the average tracks when some frames are missing
    for (unsigned long i = m_frames_to_merge.size(); i < m_n_detectors; ++i) {
      for (const auto& track : m_avg_tracks.skeletons) {
        m_tracks_to_merge[track.id].push_back(track);
      }
    }

    computeAvgTracks();

    m_frames_to_merge.clear();
    m_tracks_to_merge.clear();

    m_ok_to_publish = true;
  }

  m_frames_to_merge.push_back(std::make_pair(ros::Time(m_tracks.src_time), m_tracks.src_frame));

  for (const auto& skeleton : m_tracks.skeletons) {
    m_tracks_to_merge[skeleton.id].push_back(skeleton);
  }

  if (m_frames_to_merge.size() == m_n_detectors) {
    computeAvgTracks();

    m_frames_to_merge.clear();
    m_tracks_to_merge.clear();

    m_ok_to_publish = true;
  }
}

void hiros::track::SkeletonTracker::publishTracks(const hiros::skeletons::types::SkeletonGroup& t_tracks) const
{
  m_out_msg_pub.publish(
    skeletons::utils::toMsg(ros::Time::now(), m_frame_id, ros::Time(t_tracks.src_time), t_tracks.src_frame, t_tracks));
}

void hiros::track::SkeletonTracker::fillDetections()
{
  m_detections.skeletons.clear();

  m_detections.src_time = m_skeleton_groups_buffer.get_src_time().toSec();
  m_detections.src_frame = m_skeleton_groups_buffer.get_src_frame();
  for (const auto& skeleton : m_skeleton_groups_buffer.get_skeleton_group()->skeletons) {
    if (!utils::isEmpty(skeleton)) {
      m_detections.addSkeleton(hiros::skeletons::utils::toStruct(skeleton));
    }
  }
}

void hiros::track::SkeletonTracker::createCostMatrix()
{
  if (m_tracks.skeletons.empty() || m_detections.skeletons.empty()) {
    m_cost_matrix = cv::Mat_<double>();
    m_markers_distance_matrix = cv::Mat_<double>();
    m_orientations_distance_matrix = cv::Mat_<double>();
    return;
  }

  auto tracks_ptr = m_avg_tracks.skeletons.empty()
                      ? std::make_unique<hiros::skeletons::types::SkeletonGroup>(m_tracks)
                      : std::make_unique<hiros::skeletons::types::SkeletonGroup>(m_avg_tracks);

  m_cost_matrix =
    cv::Mat_<double>(static_cast<int>(tracks_ptr->skeletons.size()), static_cast<int>(m_detections.skeletons.size()));
  m_markers_distance_matrix =
    cv::Mat_<double>(static_cast<int>(tracks_ptr->skeletons.size()), static_cast<int>(m_detections.skeletons.size()));
  m_orientations_distance_matrix =
    cv::Mat_<double>(static_cast<int>(tracks_ptr->skeletons.size()), static_cast<int>(m_detections.skeletons.size()));

  for (int track_idx = 0; track_idx < m_cost_matrix.rows; ++track_idx) {
    for (int det_idx = 0; det_idx < m_cost_matrix.cols; ++det_idx) {

      m_markers_distance_matrix(track_idx, det_idx) =
        computeMarkersDistance(tracks_ptr->skeletons.at(static_cast<unsigned int>(track_idx)),
                               m_detections.skeletons.at(static_cast<unsigned int>(det_idx)));
      m_orientations_distance_matrix(track_idx, det_idx) =
        computeOrientationsDistance(tracks_ptr->skeletons.at(static_cast<unsigned int>(track_idx)),
                                    m_detections.skeletons.at(static_cast<unsigned int>(det_idx)));
      m_cost_matrix(track_idx, det_idx) =
        computeDistance(tracks_ptr->skeletons.at(static_cast<unsigned int>(track_idx)),
                        m_detections.skeletons.at(static_cast<unsigned int>(det_idx)));
    }
  }

  utils::replaceNans(m_cost_matrix);
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
          && (m_markers_distance_matrix[r][c] > m_params.max_markers_distance
              || m_orientations_distance_matrix[r][c] > m_params.max_orientations_distance)) {
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
        if (utils::min(m_markers_distance_matrix.col(static_cast<int>(c))) < m_params.min_markers_distance
            && utils::min(m_orientations_distance_matrix.col(static_cast<int>(c)))
                 < m_params.min_orientations_distance) {
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
      delta_t = m_skeleton_groups_buffer.get_src_time() - m_track_id_to_time_stamp_map.at(id);

      if (delta_t > m_params.max_delta_t) {
        if (m_n_detectors == 1 && m_params.filter_trajectories) {
          m_track_id_to_filter_map.erase(id);
        }

        m_tracks_to_merge.erase(id);
        m_track_id_to_time_stamp_map.erase(id);
        m_tracks.removeSkeleton(id);
        m_avg_tracks.removeSkeleton(id);
      }
    }
  }
}

double hiros::track::SkeletonTracker::computeMarkersDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                             hiros::skeletons::types::Skeleton& t_detection) const
{
  // If both the Skeletons do not have markers, then they are pure OrientationSkeletons
  if (skeletons::utils::numberOfMarkers(t_track) == 0 && skeletons::utils::numberOfMarkers(t_detection) == 0) {
    return 0;
  }

  double pos_dist = 0;
  double vel_dist = 0;
  unsigned int pos_n_mks = 0;
  unsigned int vel_n_mks = 0;
  double weight;

  if (m_params.use_marker_velocities || m_params.weight_distances_by_velocities) {
    computeVelocities(t_track,
                      t_detection,
                      (m_skeleton_groups_buffer.get_src_time() - m_track_id_to_time_stamp_map.at(t_track.id)).toSec());
  }

  for (const auto& det_mkg : t_detection.marker_groups) {
    if (t_track.hasMarkerGroup(det_mkg.id)) {
      auto& track_mkg = t_track.getMarkerGroup(det_mkg.id);

      for (const auto& det_mk : det_mkg.markers) {
        if (track_mkg.hasMarker(det_mk.id)) {
          auto& track_mk = track_mkg.getMarker(det_mk.id);

          weight = (m_params.weight_distances_by_confidences) ? det_mk.confidence : 1;
          if (m_params.weight_distances_by_velocities) {
            // The higher the velocity the lower the marker will be weighted. Raise velocity magnitude to the power of
            // -0.25 to have smoother weights: vel = 0 -> weight = 1, vel = 10 -> weight = 0.55, vel = 100 -> weight =
            // 0.32, vel = inf -> weight = 0
            weight *= std::pow(1 + hiros::skeletons::utils::magnitude(track_mk.point.velocity), -0.25);
          }

          if (m_params.use_markers) {
            pos_dist += (weight * hiros::skeletons::utils::distance(det_mk.point.position, track_mk.point.position));
            ++pos_n_mks;
          }

          if (m_params.use_marker_velocities) {
            vel_dist += (weight * hiros::skeletons::utils::distance(det_mk.point.velocity, track_mk.point.velocity));
            ++vel_n_mks;
          }
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
                                       : std::numeric_limits<double>::max();
}

double hiros::track::SkeletonTracker::computeOrientationsDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                                  hiros::skeletons::types::Skeleton& t_detection) const
{
  // If both the Skeletons do not have orientations, then they are pure MarkerSkeletons
  if (skeletons::utils::numberOfOrientations(t_track) == 0
      && skeletons::utils::numberOfOrientations(t_detection) == 0) {
    return 0;
  }

  double or_dist = 0;
  double vel_dist = 0;
  unsigned int or_n_ors = 0;
  unsigned int vel_n_ors = 0;
  double weight;

  for (const auto& det_org : t_detection.orientation_groups) {
    if (t_track.hasOrientationGroup(det_org.id)) {
      auto& track_org = t_track.getOrientationGroup(det_org.id);

      for (const auto& det_or : det_org.orientations) {
        if (track_org.hasOrientation(det_or.id)) {
          auto& track_or = track_org.getOrientation(det_or.id);

          weight = (m_params.weight_distances_by_confidences) ? det_or.confidence : 1;
          if (m_params.weight_distances_by_velocities) {
            // The higher the velocity the lower the Orientation will be weighted. Raise velocity magnitude to the
            // power of -0.25 to have smoother weights: vel = 0 -> weight = 1, vel = 10 -> weight = 0.55, vel = 100 ->
            // weight = 0.32, vel = inf -> weight = 0
            weight *= std::pow(1 + hiros::skeletons::utils::magnitude(track_or.mimu.angular_velocity), -0.25);
          }

          if (m_params.use_orientations) {
            or_dist += (weight * hiros::skeletons::utils::distance(det_or.mimu.orientation, track_or.mimu.orientation));
            ++or_n_ors;
          }

          if (m_params.use_orientation_velocities) {
            vel_dist +=
              (weight
               * hiros::skeletons::utils::distance(det_or.mimu.angular_velocity, track_or.mimu.angular_velocity));
            ++vel_n_ors;
          }
        }
      }
    }
  }

  // Divide the average distance by n_ors^0.25 to prefer track-detection matches that have an higher number of
  // Orientations in common
  if (or_n_ors > 0) {
    or_dist /= std::pow(or_n_ors, 1.25);
  }
  if (vel_n_ors > 0) {
    vel_dist /= std::pow(vel_n_ors, 1.25);
  }

  return ((or_n_ors + vel_n_ors) > 0) ? or_dist + m_params.velocity_weight * vel_dist
                                      : std::numeric_limits<double>::max();
}

double hiros::track::SkeletonTracker::computeDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                      hiros::skeletons::types::Skeleton& t_detection) const
{
  return computeMarkersDistance(t_track, t_detection) + computeOrientationsDistance(t_track, t_detection);
}

void hiros::track::SkeletonTracker::initializeVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton) const
{
  for (auto& mkg : t_skeleton.marker_groups) {
    for (auto& mk : mkg.markers) {
      mk.point.velocity = hiros::skeletons::types::Velocity();
      mk.point.acceleration = hiros::skeletons::types::Acceleration();
    }
  }
}

void hiros::track::SkeletonTracker::computeVelAndAcc(const hiros::skeletons::types::Skeleton& t_track,
                                                     hiros::skeletons::types::Skeleton& t_detection,
                                                     const double& t_dt) const
{
  for (auto& det_mkg : t_detection.marker_groups) {
    if (t_track.hasMarkerGroup(det_mkg.id)) {
      auto& track_mkg = t_track.getMarkerGroup(det_mkg.id);
      for (auto& det_mk : det_mkg.markers) {
        if (track_mkg.hasMarker(det_mk.id)) {
          auto& track_mk = track_mkg.getMarker(det_mk.id);
          det_mk.point.velocity = computeVelocity(track_mk.point, det_mk.point, t_dt);
          det_mk.point.acceleration = computeAcceleration(track_mk.point, det_mk.point, t_dt);
        }
      }
    }
  }
}

void hiros::track::SkeletonTracker::computeVelocities(const hiros::skeletons::types::Skeleton& t_track,
                                                      hiros::skeletons::types::Skeleton& t_detection,
                                                      const double& t_dt) const
{
  for (auto& det_mkg : t_detection.marker_groups) {
    if (t_track.hasMarkerGroup(det_mkg.id)) {
      auto& track_mkg = t_track.getMarkerGroup(det_mkg.id);
      for (auto& det_mk : det_mkg.markers) {
        if (track_mkg.hasMarker(det_mk.id)) {
          auto& track_mk = track_mkg.getMarker(det_mk.id);
          det_mk.point.velocity = computeVelocity(track_mk.point, det_mk.point, t_dt);
        }
      }
    }
  }
}

hiros::skeletons::types::Velocity
hiros::track::SkeletonTracker::computeVelocity(const hiros::skeletons::types::Point& t_prev,
                                               const hiros::skeletons::types::Point& t_curr,
                                               const double& t_dt) const
{
  return hiros::skeletons::types::Velocity((t_curr.position.x() - t_prev.position.x()) / t_dt,
                                           (t_curr.position.y() - t_prev.position.y()) / t_dt,
                                           (t_curr.position.z() - t_prev.position.z()) / t_dt);
}

hiros::skeletons::types::Acceleration
hiros::track::SkeletonTracker::computeAcceleration(const hiros::skeletons::types::Point& t_prev,
                                                   const hiros::skeletons::types::Point& t_curr,
                                                   const double& t_dt) const
{
  return hiros::skeletons::types::Acceleration((t_curr.velocity.x() - t_prev.velocity.x()) / t_dt,
                                               (t_curr.velocity.y() - t_prev.velocity.y()) / t_dt,
                                               (t_curr.velocity.z() - t_prev.velocity.z()) / t_dt);
}

void hiros::track::SkeletonTracker::updateDetectedTrack(const unsigned int& t_track_idx, const unsigned int& t_det_idx)
{
  if (utils::matchMunkres(m_munkres_matrix, t_track_idx, t_det_idx)) {
    int id = m_tracks.skeletons.at(t_track_idx).id;

    if (m_n_detectors == 1 && !m_params.filter_trajectories) {
      computeVelAndAcc(m_tracks.skeletons.at(t_track_idx),
                       m_detections.skeletons.at(t_det_idx),
                       (m_skeleton_groups_buffer.get_src_time() - m_track_id_to_time_stamp_map.at(id)).toSec());
    }

    m_track_id_to_time_stamp_map.at(id) = m_skeleton_groups_buffer.get_src_time();

    m_tracks.skeletons.at(t_track_idx) = m_detections.skeletons.at(t_det_idx);
    m_tracks.skeletons.at(t_track_idx).id = id;

    if (m_n_detectors == 1 && m_params.filter_trajectories) {
      m_track_id_to_filter_map.at(id).filter(m_tracks.skeletons.at(t_track_idx),
                                             m_track_id_to_time_stamp_map.at(id).toSec());
    }
  }
}

void hiros::track::SkeletonTracker::addNewTrack(const hiros::skeletons::types::Skeleton& t_detection)
{
  if (skeletons::utils::numberOfMarkers(t_detection) >= static_cast<unsigned int>(m_params.min_joints)
      || skeletons::utils::numberOfOrientations(t_detection) >= static_cast<unsigned int>(m_params.min_joints)) {
    m_tracks.addSkeleton(t_detection);
    m_tracks.skeletons.back().id = ++m_last_track_id;
    initializeVelAndAcc(m_tracks.skeletons.back());

    m_track_id_to_time_stamp_map.emplace(m_last_track_id, m_skeleton_groups_buffer.get_src_time());

    if (m_n_detectors == 1 && m_params.filter_trajectories) {
      m_track_id_to_filter_map.emplace(m_tracks.skeletons.back().id,
                                       hiros::track::Filter(m_tracks.skeletons.back(),
                                                            m_track_id_to_time_stamp_map.at(m_last_track_id).toSec(),
                                                            m_params.filter_cutoff_frequency));
    }
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

void hiros::track::SkeletonTracker::computeAvgTracks()
{
  auto prev_avg_tracks = m_avg_tracks;

  m_avg_tracks.skeletons.clear();

  m_avg_tracks.src_time = computeAvgSrcTime();

  for (const auto& track : m_tracks_to_merge) {
    auto avg_track = computeAvgTrack(track.second);
    bool empty_prev_avg_track = !prev_avg_tracks.hasSkeleton(avg_track.id);

    if (m_params.filter_trajectories) {
      if (empty_prev_avg_track) {
        // new track
        m_track_id_to_filter_map.emplace(
          avg_track.id, hiros::track::Filter(avg_track, m_avg_tracks.src_time, m_params.filter_cutoff_frequency));
      }
      else {
        m_track_id_to_filter_map.at(avg_track.id).filter(avg_track, m_avg_tracks.src_time);
      }
    }
    else {
      if (empty_prev_avg_track) {
        // new track
        initializeVelAndAcc(avg_track);
      }
      else {
        computeVelAndAcc(
          prev_avg_tracks.getSkeleton(avg_track.id), avg_track, (m_avg_tracks.src_time - prev_avg_tracks.src_time));
      }
    }

    m_avg_tracks.addSkeleton(avg_track);
  }
}

double hiros::track::SkeletonTracker::computeAvgSrcTime() const
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

hiros::skeletons::types::Skeleton
hiros::track::SkeletonTracker::computeAvgTrack(const std::vector<hiros::skeletons::types::Skeleton>& t_tracks) const
{
  hiros::skeletons::types::Skeleton avg_sk = t_tracks.front();

  for (unsigned int i = 1; i < t_tracks.size(); ++i) {
    utils::merge(avg_sk, t_tracks.at(i), i, 1, true);
  }

  return avg_sk;
}
