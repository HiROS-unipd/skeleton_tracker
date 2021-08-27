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
  double max_delta_t = 0, fixed_delay = 0;

  m_nh.getParam("camera_frequency", m_params.camera_frequency);
  if (m_params.camera_frequency <= 0) {
    m_params.camera_frequency = std::numeric_limits<double>::min();
  }
  m_nh.getParam("fixed_delay", fixed_delay);
  m_params.fixed_delay = ros::Duration(fixed_delay);
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
  ros::spin();
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

void hiros::track::SkeletonTracker::detectorCallback(hiros_skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  checkFrameIdConsistency(t_skeleton_group_msg);

  if (t_skeleton_group_msg->src_time <= getPreviousSrcTime()) {
    return;
  }

  addNewSkeletonGroupToBuffer(t_skeleton_group_msg);

  while (!m_skeleton_groups_buffer.empty()
         && (t_skeleton_group_msg->src_time - m_skeleton_groups_buffer.get_src_time()) >= m_params.fixed_delay) {
    trackOldestFrame();
    mergeTracks();
  }
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

void hiros::track::SkeletonTracker::addNewSkeletonGroupToBuffer(
  hiros_skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  m_skeleton_groups_buffer.push_back(t_skeleton_group_msg);
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

void hiros::track::SkeletonTracker::mergeTracks()
{
  for (auto& pair : m_tracks_to_merge) {
    auto last_track = utils::getSkeletonFromId(m_tracks, pair.first);
    auto last_src_frame = last_track->src_frame;
    auto last_src_time = last_track->src_time;

    // If one of the tracks to merge has its src_frame equal to the one of the latest received track, and its src_time
    // is lower than the one of the latest received track, then the current track belongs to a new frame, and it is time
    // to compute the average track
    if (std::find_if(
          pair.second.begin(),
          pair.second.end(),
          [&](const auto& elem) { return (elem.src_frame == last_src_frame && elem.src_time < last_src_time); })
        != pair.second.end()) {
      computeAvgTrack(pair.first);

      if (m_ok_to_publish) {
        publishTracks();
      }
    }
  }

  for (auto& track : m_tracks) {
    auto id = track.skeleton.id;

    // If the delta between the src_time of the latest received track and the oldest track to merge is greater than half
    // of the camera's dt between frames (multiplied by 1.1 to take into account possible fluctuations of the camera's
    // frequency), then the track belongs to a new frame, and it is time to compute the average track
    if (!m_tracks_to_merge[id].empty()
        && (track.src_time - m_tracks_to_merge[id].front().src_time).toSec() > 1.1 / m_params.camera_frequency / 2.) {
      computeAvgTrack(id);

      if (m_ok_to_publish) {
        publishTracks();
      }
    }

    m_tracks_to_merge[id].push_back(track);

    // If the number of tracks to merge is equal to the number of detectors, then the next track will necessarily belong
    // to a new frame, and we can compute the current frame's average track withouth having to wait for the next frame
    if (m_tracks_to_merge.at(id).size() == m_n_detectors) {
      computeAvgTrack(id);

      if (m_ok_to_publish) {
        publishTracks();
      }
    }
  }
}

ros::Time hiros::track::SkeletonTracker::getPreviousSrcTime() const
{
  // Return the source time of the latest avelable tracks
  return !m_tracks.empty()
           ? (std::max_element(m_tracks.begin(),
                               m_tracks.end(),
                               [](const auto& t1, const auto& t2) { return t1.src_time < t2.src_time; }))
               ->src_time
           : ros::Time();
}

ros::Time hiros::track::SkeletonTracker::getCurrentSrcTime() const
{
  // If m_detections is already filled, then the current src_time is the max among the detections' src_times
  if (!m_detections.empty()) {
    return (std::max_element(m_detections.begin(),
                             m_detections.end(),
                             [](const auto& t1, const auto& t2) { return t1.src_time < t2.src_time; }))
      ->src_time;
  }
  // Else, if the skeleton_group buffer is not empty, the current src_time is the one from the oldest frame in the
  // buffer
  else if (!m_skeleton_groups_buffer.empty()) {
    return m_skeleton_groups_buffer.get_src_time();
  }

  return ros::Time();
}

void hiros::track::SkeletonTracker::publishTracks()
{
  std::sort(m_avg_tracks.begin(), m_avg_tracks.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.skeleton.id < rhs.skeleton.id;
  });

  m_out_msg_pub.publish(skeletons::utils::toMsg(
    ros::Time::now(), m_frame_id, ros::Time(computeAvgSrcTime(m_avg_tracks)), "", toSkeletonGroup(m_avg_tracks)));

  m_ok_to_publish = false;
  m_avg_tracks.clear();
}

hiros::skeletons::types::SkeletonGroup
hiros::track::SkeletonTracker::toSkeletonGroup(const std::vector<utils::StampedSkeleton>& t_skel_vector) const
{
  hiros::skeletons::types::SkeletonGroup msg;
  for (const auto& skel : t_skel_vector) {
    msg.addSkeleton(skel.skeleton);
  }

  return msg;
}

void hiros::track::SkeletonTracker::fillDetections()
{
  m_detections.clear();

  for (const auto& skeleton : m_skeleton_groups_buffer.get_skeleton_group()->skeletons) {
    if (!utils::isEmpty(skeleton)) {
      m_detections.emplace_back(hiros::skeletons::utils::toStruct(skeleton),
                                m_skeleton_groups_buffer.get_src_time(),
                                m_skeleton_groups_buffer.get_src_frame());
    }
  }

  eraseOldSkeletonGroupFromBuffer();
}

void hiros::track::SkeletonTracker::createCostMatrix()
{
  if (m_tracks.empty() || m_detections.empty()) {
    m_cost_matrix = cv::Mat_<double>();
    m_markers_distance_matrix = cv::Mat_<double>();
    m_orientations_distance_matrix = cv::Mat_<double>();
    return;
  }

  auto tracks = getLatestAvailableTracks();

  m_cost_matrix = cv::Mat_<double>(static_cast<int>(tracks.size()), static_cast<int>(m_detections.size()));
  m_markers_distance_matrix = cv::Mat_<double>(static_cast<int>(tracks.size()), static_cast<int>(m_detections.size()));
  m_orientations_distance_matrix =
    cv::Mat_<double>(static_cast<int>(tracks.size()), static_cast<int>(m_detections.size()));

  for (int track_idx = 0; track_idx < m_cost_matrix.rows; ++track_idx) {
    for (int det_idx = 0; det_idx < m_cost_matrix.cols; ++det_idx) {
      m_markers_distance_matrix(track_idx, det_idx) =
        computeMarkersDistance(tracks.at(static_cast<unsigned int>(track_idx)).skeleton,
                               m_detections.at(static_cast<unsigned int>(det_idx)).skeleton);
      m_orientations_distance_matrix(track_idx, det_idx) =
        computeOrientationsDistance(tracks.at(static_cast<unsigned int>(track_idx)).skeleton,
                                    m_detections.at(static_cast<unsigned int>(det_idx)).skeleton);
      m_cost_matrix(track_idx, det_idx) =
        computeWeightedDistance(tracks.at(static_cast<unsigned int>(track_idx)).skeleton,
                                m_detections.at(static_cast<unsigned int>(det_idx)).skeleton);
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
          && (m_markers_distance_matrix[r][c] > m_params.max_markers_distance
              || m_orientations_distance_matrix[r][c] > m_params.max_orientations_distance)) {
        m_munkres_matrix(r, c) = 0;
      }
    }
  }
}

void hiros::track::SkeletonTracker::updateDetectedTracks()
{
  for (unsigned int track_idx = 0; track_idx < m_tracks.size(); ++track_idx) {
    for (unsigned int det_idx = 0; det_idx < m_detections.size(); ++det_idx) {
      updateDetectedTrack(track_idx, det_idx);
    }
  }
}

void hiros::track::SkeletonTracker::addNewTracks()
{
  if (m_detections.empty()) {
    return;
  }

  if (m_cost_matrix.empty()) {
    for (const auto& detection : m_detections) {
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
          utils::merge(m_tracks.at(row), m_detections.at(c));
        }
        else {
          addNewTrack(m_detections.at(c));
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

  for (int i = static_cast<int>(m_tracks.size()) - 1; i >= 0; --i) {
    track_idx = static_cast<unsigned int>(i);

    if (unassociatedTrack(track_idx)) {
      id = m_tracks.at(track_idx).skeleton.id;
      delta_t = getCurrentSrcTime() - m_tracks.at(track_idx).src_time;

      if (delta_t > m_params.max_delta_t) {
        m_tracks_to_merge.erase(id);
        m_tracks.erase(std::remove_if(m_tracks.begin(),
                                      m_tracks.end(),
                                      [&](const auto& track) { return (track.skeleton.id == id); }),
                       m_tracks.end());
        m_avg_tracks.erase(std::remove_if(m_avg_tracks.begin(),
                                          m_avg_tracks.end(),
                                          [&](const auto& avg_track) { return (avg_track.skeleton.id == id); }),
                           m_avg_tracks.end());
        m_track_filters.erase(id);
      }
    }
  }
}

void hiros::track::SkeletonTracker::eraseOldSkeletonGroupFromBuffer()
{
  m_skeleton_groups_buffer.pop_back();
}

std::vector<hiros::track::utils::StampedSkeleton> hiros::track::SkeletonTracker::getLatestAvailableTracks() const
{
  if (m_avg_tracks.size() > m_tracks.size()) {
    ROS_FATAL_STREAM("Error: avg_tracks size > tracks size. This should not happen. Shutting down");
    ros::shutdown();
  }
  // If the avg_tracks' size is equal to the tracks' size, then I have already computed an avg_track for each track,
  // and can directly return the avg_tracks
  else if (m_avg_tracks.size() == m_tracks.size()) {
    return m_avg_tracks;
  }
  // If the avg_tracks are empty, then I only have the raw tracks
  else if (m_avg_tracks.empty()) {
    return m_tracks;
  }

  // If I have both, and the avg_tracks' size is lower the the tracks' size, then I take the available
  // avg_tracks and fill the missing ones with the raw tracks
  auto ret_tracks = m_tracks;
  for (const auto& avg_track : m_avg_tracks) {
    auto track_ptr = utils::getSkeletonFromId(ret_tracks, avg_track.skeleton.id);
    if (track_ptr != nullptr) {
      *track_ptr = avg_track;
    }
  }
  return ret_tracks;
}

double hiros::track::SkeletonTracker::computeMarkersDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                             hiros::skeletons::types::Skeleton& t_detection) const
{
  double dist = 0;
  unsigned int n_mks = 0;

  for (const auto& det_mkg : t_detection.marker_groups) {
    if (t_track.hasMarkerGroup(det_mkg.id)) {
      auto& track_mkg = t_track.getMarkerGroup(det_mkg.id);

      for (const auto& det_mk : det_mkg.markers) {
        if (track_mkg.hasMarker(det_mk.id)) {
          auto& track_mk = track_mkg.getMarker(det_mk.id);

          dist += hiros::skeletons::utils::distance(det_mk.point.position, track_mk.point.position);
          ++n_mks;
        }
      }
    }
  }

  return (n_mks > 0) ? dist / n_mks : std::numeric_limits<double>::max();
}

double hiros::track::SkeletonTracker::computeOrientationsDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                                  hiros::skeletons::types::Skeleton& t_detection) const
{
  double dist = 0;
  unsigned int n_ors = 0;

  for (const auto& det_org : t_detection.orientation_groups) {
    if (t_track.hasOrientationGroup(det_org.id)) {
      auto& track_org = t_track.getOrientationGroup(det_org.id);

      for (const auto& det_or : det_org.orientations) {
        if (track_org.hasOrientation(det_or.id)) {
          auto& track_or = track_org.getOrientation(det_or.id);

          dist += hiros::skeletons::utils::distance(det_or.mimu.orientation, track_or.mimu.orientation);
          ++n_ors;
        }
      }
    }
  }

  return (n_ors > 0) ? dist / n_ors : std::numeric_limits<double>::max();
}

double
hiros::track::SkeletonTracker::computeWeightedMarkersDistance(const hiros::skeletons::types::Skeleton& t_track,
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
    computeVelocities(
      t_track,
      t_detection,
      (m_skeleton_groups_buffer.get_src_time() - utils::getSkeletonFromId(m_tracks, t_track.id)->src_time).toSec());
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

double
hiros::track::SkeletonTracker::computeWeightedOrientationsDistance(const hiros::skeletons::types::Skeleton& t_track,
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

double hiros::track::SkeletonTracker::computeWeightedDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                              hiros::skeletons::types::Skeleton& t_detection) const
{
  return computeWeightedMarkersDistance(t_track, t_detection)
         + computeWeightedOrientationsDistance(t_track, t_detection);
}

void hiros::track::SkeletonTracker::initializeVelAndAcc(utils::StampedSkeleton& t_skeleton) const
{
  for (auto& mkg : t_skeleton.skeleton.marker_groups) {
    for (auto& mk : mkg.markers) {
      mk.point.velocity = hiros::skeletons::types::Velocity(0, 0, 0);
      mk.point.acceleration = hiros::skeletons::types::Acceleration(0, 0, 0);
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
    int id = m_tracks.at(t_track_idx).skeleton.id;
    m_tracks.at(t_track_idx) = m_detections.at(t_det_idx);
    m_tracks.at(t_track_idx).skeleton.id = id;
  }
}

void hiros::track::SkeletonTracker::addNewTrack(const utils::StampedSkeleton& t_detection)
{
  if (skeletons::utils::numberOfMarkers(t_detection.skeleton) >= static_cast<unsigned int>(m_params.min_joints)
      || skeletons::utils::numberOfOrientations(t_detection.skeleton)
           >= static_cast<unsigned int>(m_params.min_joints)) {
    m_tracks.push_back(t_detection);
    m_tracks.back().skeleton.id = ++m_last_track_id;
    initializeVelAndAcc(m_tracks.back());
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

double hiros::track::SkeletonTracker::computeAvgSrcTime(
  const std::vector<hiros::track::utils::StampedSkeleton>& t_skeletons) const
{
  if (t_skeletons.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double sum{0};
  unsigned int n_elems{0};
  for (const auto& skel : t_skeletons) {
    if (!utils::isEmpty(skel.skeleton)) {
      sum += skel.src_time.toSec();
      ++n_elems;
    }
  }

  return sum / n_elems;
}

void hiros::track::SkeletonTracker::computeAvgTrack(const int& t_id)
{
  // Compute the avg track for id t_id
  auto avg_track = m_tracks_to_merge.at(t_id).front();
  avg_track.src_time = ros::Time(computeAvgSrcTime(m_tracks_to_merge.at(t_id)));

  for (unsigned int i = 1; i < m_tracks_to_merge.at(t_id).size(); ++i) {
    utils::merge(avg_track, m_tracks_to_merge.at(t_id).at(i), i, 1, true);
  }

  // Get the previous avg track for id t_id
  auto prev_avg_track = utils::getSkeletonFromId(m_prev_avg_tracks, avg_track.skeleton.id);

  if (m_params.filter_trajectories) {
    m_track_filters[avg_track.skeleton.id].filter(avg_track, m_params.filter_cutoff_frequency);
  }
  else {
    // Compute vel and acc only if we have previous data
    if (prev_avg_track != nullptr) {
      computeVelAndAcc(
        prev_avg_track->skeleton, avg_track.skeleton, (avg_track.src_time - prev_avg_track->src_time).toSec());
    }
  }

  addToAvgTracks(avg_track);

  // Now we can update m_prev_avg_tracks that will be used on the next frame, and clean the tracks to merge relative
  // to id t_id
  m_prev_avg_tracks = m_avg_tracks;
  m_tracks_to_merge.at(t_id).clear();

  if (m_avg_tracks.size() > m_tracks.size()) {
    ROS_FATAL_STREAM("Error: avg_tracks size > tracks size. This should not happen. Shutting down");
    ros::shutdown();
  }

  m_ok_to_publish = (m_avg_tracks.size() == m_tracks.size());
}

void hiros::track::SkeletonTracker::addToAvgTracks(const utils::StampedSkeleton& t_track)
{
  auto idx = utils::getSkeletonIndexFromId(m_avg_tracks, t_track.skeleton.id);

  if (idx == -1) {
    m_avg_tracks.push_back(t_track);
  }
  else {
    m_avg_tracks.at(static_cast<unsigned int>(idx)) = t_track;
  }
}
