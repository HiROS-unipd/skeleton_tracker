// ROS dependencies
#include <ros/ros.h>
#include <std_msgs/Header.h>

// Custom Ros Message dependencies
#include "skeleton_msgs/SkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/Tracker.h"
#include "skeleton_tracker/utils.h"

hiros::track::Tracker::Tracker()
  : m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_last_track_id(-1)
  , m_configured(false)
{
  m_detections = skeletons::types::SkeletonGroup();
  m_tracks = skeletons::types::SkeletonGroup();
}

hiros::track::Tracker::~Tracker() {}

void hiros::track::Tracker::configure()
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
  m_nh.getParam("min_keypoints", m_params.min_keypoints);
  m_nh.getParam("min_distance", m_params.min_distance);
  m_nh.getParam("max_distance", m_params.max_distance);
  m_nh.getParam("max_delta_t", max_delta_t);
  m_params.max_delta_t = ros::Duration(max_delta_t);
  m_nh.getParam("use_keypoint_positions", m_params.use_keypoint_positions);
  m_nh.getParam("use_keypoint_velocities", m_params.use_keypoint_velocities);
  m_nh.getParam("velocity_weight", m_params.velocity_weight);
  m_nh.getParam("weight_distances_by_confidences", m_params.weight_distances_by_confidences);
  m_nh.getParam("weight_distances_by_velocities", m_params.weight_distances_by_velocities);

  m_nh.getParam("filter_keypoint_trajectories", m_params.filter_keypoint_trajectories);
  m_nh.getParam("filter_cutoff_frequency", m_params.filter_cutoff_frequency);

  if (m_params.filter_keypoint_trajectories && (m_params.filter_cutoff_frequency <= 0.0)) {
    ROS_FATAL_STREAM("Cutoff frequency must be greater than 0 Hz. Unable to continue");
    ros::shutdown();
  }

  if (!m_params.use_keypoint_positions && !m_params.use_keypoint_velocities) {
    ROS_FATAL_STREAM("Position based distance and/or velocity based distance must be enabled. Unable to continue");
    ros::shutdown();
  }

  m_configured = true;
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker...CONFIGURED" << BASH_MSG_RESET);
}

void hiros::track::Tracker::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Tracker...Starting");

  if (!m_configured) {
    configure();
  }

  setupRosTopics();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker...RUNNING" << BASH_MSG_RESET);
}

void hiros::track::Tracker::stop()
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

std::string
hiros::track::Tracker::extractImageQualityFromTopicNames(const std::vector<std::string>& t_topic_names) const
{
  std::vector<std::string> image_qualities;
  image_qualities.reserve(t_topic_names.size());

  for (auto& topic_name : t_topic_names) {
    auto tmp1 = topic_name.find_last_of("/");
    std::string tmp2 = topic_name.substr(0, tmp1);
    auto tmp3 = tmp2.find_last_of("/");
    image_qualities.push_back(topic_name.substr(tmp3 + 1, tmp1 - tmp3 - 1));
  }

  return (std::adjacent_find(image_qualities.begin(), image_qualities.end(), std::not_equal_to<std::string>())
          == image_qualities.end())
           ? image_qualities.front()
           : std::string();
}

void hiros::track::Tracker::setupRosTopics()
{
  for (auto& topic : m_params.in_skeleton_group_topics) {
    m_in_skeleton_group_subs.push_back(m_nh.subscribe(topic, 1, &Tracker::detectorCallback, this));
  }

  for (auto& sub : m_in_skeleton_group_subs) {
    while (sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
      ROS_WARN_STREAM_THROTTLE(2, m_node_namespace << " No input messages on skeleton group topic(s)");
    }
  }

  std::string out_msg_topic = m_params.out_msg_topic_name;
  std::string image_quality = extractImageQualityFromTopicNames(m_params.in_skeleton_group_topics);
  if (!image_quality.empty()) {
    out_msg_topic.insert(0, image_quality + "/");
  }
  m_out_msg_pub = m_nh.advertise<skeleton_msgs::SkeletonGroup>(out_msg_topic, 1);
}

void hiros::track::Tracker::checkFrameIdConsistency(const skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  if (m_received_frames.size() < m_n_detectors) {
    auto src_frame = t_skeleton_group_msg->src_frame;

    if (std::find_if(m_received_frames.begin(),
                     m_received_frames.end(),
                     [&src_frame](const std::pair<std::string, std::string>& elem) { return elem.second == src_frame; })
        == m_received_frames.end()) {
      m_received_frames.push_back(
        std::make_pair(t_skeleton_group_msg->src_frame, t_skeleton_group_msg->header.frame_id));
    }

    if (m_received_frames.size() == m_n_detectors) {
      m_frame_id = m_received_frames.front().second;

      for (auto& frame_pair : m_received_frames) {
        if (frame_pair.second != m_frame_id) {
          ROS_FATAL_STREAM("Error. Data must be expressed w.r.t the same reference frame");
          ros::shutdown();
          exit(EXIT_FAILURE);
        }
      }
    }
  }
}

void hiros::track::Tracker::detectorCallback(const skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  checkFrameIdConsistency(t_skeleton_group_msg);

  if (t_skeleton_group_msg->src_time <= getPreviousSourceTime()) {
    return;
  }

  addNewSkeletonGroupToBuffer(t_skeleton_group_msg);

  if (m_params.in_skeleton_group_topics.size() == 1
      || (t_skeleton_group_msg->src_time.toSec() - m_skeleton_groups_buffer.begin()->second.src_time)
           >= m_params.fixed_delay) {
    track();
    publishTracks(m_tracks);
    eraseOldSkeletonGroupFromBuffer();
  }
}

void hiros::track::Tracker::track()
{
  fillDetections();
  createCostMatrix();
  solveMunkres();
  removeDistantMatches();
  updateDetectedTracks();
  addNewTracks();
  removeUnassociatedTracks();
}

ros::Time hiros::track::Tracker::getPreviousSourceTime() const
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

void hiros::track::Tracker::addNewSkeletonGroupToBuffer(const skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  m_skeleton_groups_buffer.emplace(t_skeleton_group_msg->src_time,
                                   hiros::skeletons::utils::toStruct(*t_skeleton_group_msg));
}

void hiros::track::Tracker::eraseOldSkeletonGroupFromBuffer()
{
  m_skeleton_groups_buffer.erase(m_skeleton_groups_buffer.begin());
}

void hiros::track::Tracker::publishTracks(const hiros::skeletons::types::SkeletonGroup t_tracks) const
{
  m_out_msg_pub.publish(
    skeletons::utils::toMsg(ros::Time::now(), m_frame_id, ros::Time(t_tracks.src_time), t_tracks.src_frame, t_tracks));
}

void hiros::track::Tracker::fillDetections()
{
  m_detections.skeletons.clear();

  m_detections.src_time = m_skeleton_groups_buffer.begin()->second.src_time;
  m_detections.src_frame = m_skeleton_groups_buffer.begin()->second.src_frame;
  for (auto& skeleton : m_skeleton_groups_buffer.begin()->second.skeletons) {
    if (!utils::isEmpty(skeleton)) {
      m_detections.skeletons.push_back(skeleton);
    }
  }
}

void hiros::track::Tracker::createCostMatrix()
{
  if (m_tracks.skeletons.empty() || m_detections.skeletons.empty()) {
    m_cost_matrix = cv::Mat_<double>();
    return;
  }

  m_cost_matrix =
    cv::Mat_<double>(static_cast<int>(m_tracks.skeletons.size()), static_cast<int>(m_detections.skeletons.size()));

  for (unsigned int track_idx = 0; track_idx < m_tracks.skeletons.size(); ++track_idx) {
    for (unsigned int det_idx = 0; det_idx < m_detections.skeletons.size(); ++det_idx) {
      m_cost_matrix(static_cast<int>(track_idx), static_cast<int>(det_idx)) =
        computeDistance(m_tracks.skeletons.at(track_idx), m_detections.skeletons.at(det_idx));
    }
  }

  utils::replaceNans(m_cost_matrix);
}

void hiros::track::Tracker::solveMunkres()
{
  // Munkres: rows = tracks, cols = detections
  m_munkres_matrix = m_munkres.solve(m_cost_matrix);
}

void hiros::track::Tracker::removeDistantMatches()
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

void hiros::track::Tracker::updateDetectedTracks()
{
  for (unsigned int track_idx = 0; track_idx < m_tracks.skeletons.size(); ++track_idx) {
    for (unsigned int det_idx = 0; det_idx < m_detections.skeletons.size(); ++det_idx) {
      updateDetectedTrack(track_idx, det_idx);
    }
  }
}

void hiros::track::Tracker::addNewTracks()
{
  if (std::isnan(m_tracks.src_time)) {
    m_tracks.src_time = m_detections.src_time;
  }

  if (m_detections.skeletons.empty()) {
    return;
  }

  if (m_cost_matrix.empty()) {
    for (auto& detection : m_detections.skeletons) {
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
          utils::merge(m_tracks.skeletons.at(row), m_detections.skeletons.at(c));
        }
        else {
          addNewTrack(m_detections.skeletons.at(c));
        }
      }
    }
  }
}

void hiros::track::Tracker::removeUnassociatedTracks()
{
  int id;
  ros::Duration delta_t;

  for (unsigned int track_idx = 0, idx_to_erase = 0; track_idx < m_tracks.skeletons.size();
       ++track_idx, ++idx_to_erase) {
    if (unassociatedTrack(track_idx)) {
      id = m_tracks.skeletons.at(idx_to_erase).id;
      delta_t = m_skeleton_groups_buffer.begin()->first - m_track_id_to_time_stamp_map.at(id);

      if (delta_t > m_params.max_delta_t) {
        if (m_params.filter_keypoint_trajectories) {
          m_track_id_to_filter_map.erase(id);
        }
        m_track_id_to_time_stamp_map.erase(id);
        m_tracks.skeletons.erase(m_tracks.skeletons.begin() + idx_to_erase--);
      }
    }
  }
}

double hiros::track::Tracker::computeDistance(const hiros::skeletons::types::Skeleton& t_track,
                                              hiros::skeletons::types::Skeleton& t_detection)
{
  double pos_dist = 0;
  double vel_dist = 0;
  unsigned int pos_n_kps = 0;
  unsigned int vel_n_kps = 0;
  double weight;

  if (m_params.use_keypoint_velocities || m_params.weight_distances_by_velocities) {
    computeVelocities(t_track,
                      t_detection,
                      (m_skeleton_groups_buffer.begin()->first - m_track_id_to_time_stamp_map.at(t_track.id)).toSec());
  }

  for (auto& det_kpg : t_detection.skeleton_parts) {
    for (auto& det_kp : det_kpg.keypoints) {
      auto track_kp = utils::findKeypoint(t_track, det_kpg.id, det_kp.id);

      if (track_kp != nullptr) {
        weight = (m_params.weight_distances_by_confidences) ? det_kp.confidence : 1;

        if (m_params.weight_distances_by_velocities) {
          // Elevate velocity magnitude to the power of 0.25 to have smoother weights
          weight *= std::pow(1 + utils::magnitude(track_kp->point.velocity), -0.25);
        }

        if (m_params.use_keypoint_positions) {
          pos_dist += (weight * utils::distance(det_kp.point.position, track_kp->point.position));
          ++pos_n_kps;
        }

        if (m_params.use_keypoint_velocities) {
          vel_dist += (weight * utils::distance(det_kp.point.velocity, track_kp->point.velocity));
          ++vel_n_kps;
        }
      }
    }
  }

  // Divide the average distance by n_kps^0.25 to prefer track-detection matches that have an higher number of
  // keypoints in common
  if (pos_n_kps > 0) {
    pos_dist /= std::pow(pos_n_kps, 1.25);
  }
  if (vel_n_kps > 0) {
    vel_dist /= std::pow(vel_n_kps, 1.25);
  }

  return ((pos_n_kps + vel_n_kps) > 0) ? pos_dist + m_params.velocity_weight * vel_dist
                                       : std::numeric_limits<double>::quiet_NaN();
}

void hiros::track::Tracker::initializeVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton)
{
  for (auto& kpg : t_skeleton.skeleton_parts) {
    for (auto& kp : kpg.keypoints) {
      kp.point.velocity = hiros::skeletons::types::Velocity();
      kp.point.acceleration = hiros::skeletons::types::Acceleration();
    }
  }
}

void hiros::track::Tracker::computeVelAndAcc(const hiros::skeletons::types::Skeleton& t_track,
                                             hiros::skeletons::types::Skeleton& t_detection,
                                             const double& t_dt)
{
  for (auto& det_kpg : t_detection.skeleton_parts) {
    for (auto& det_kp : det_kpg.keypoints) {
      auto track_kp = utils::findKeypoint(t_track, det_kpg.id, det_kp.id);

      if (track_kp != nullptr) {
        det_kp.point.velocity = computeVelocity(track_kp->point, det_kp.point, t_dt);
        det_kp.point.acceleration = computeAcceleration(track_kp->point, det_kp.point, t_dt);
      }
    }
  }
}

void hiros::track::Tracker::computeVelocities(const hiros::skeletons::types::Skeleton& t_track,
                                              hiros::skeletons::types::Skeleton& t_detection,
                                              const double& t_dt)
{
  for (auto& det_kpg : t_detection.skeleton_parts) {
    for (auto& det_kp : det_kpg.keypoints) {
      auto track_kp = utils::findKeypoint(t_track, det_kpg.id, det_kp.id);

      if (track_kp != nullptr) {
        det_kp.point.velocity = computeVelocity(track_kp->point, det_kp.point, t_dt);
      }
    }
  }
}

hiros::skeletons::types::Velocity hiros::track::Tracker::computeVelocity(const hiros::skeletons::types::Point& t_prev,
                                                                         const hiros::skeletons::types::Point& t_curr,
                                                                         const double& t_dt)
{
  return hiros::skeletons::types::Velocity((t_curr.position.x - t_prev.position.x) / t_dt,
                                           (t_curr.position.y - t_prev.position.y) / t_dt,
                                           (t_curr.position.z - t_prev.position.z) / t_dt);
}

hiros::skeletons::types::Acceleration
hiros::track::Tracker::computeAcceleration(const hiros::skeletons::types::Point& t_prev,
                                           const hiros::skeletons::types::Point& t_curr,
                                           const double& t_dt)
{
  return hiros::skeletons::types::Acceleration((t_curr.velocity.x - t_prev.velocity.x) / t_dt,
                                               (t_curr.velocity.y - t_prev.velocity.y) / t_dt,
                                               (t_curr.velocity.z - t_prev.velocity.z) / t_dt);
}

void hiros::track::Tracker::updateDetectedTrack(const unsigned int& t_track_idx, const unsigned int& t_det_idx)
{
  if (m_munkres.match(t_track_idx, t_det_idx)) {
    int id = m_tracks.skeletons.at(t_track_idx).id;

    if (!m_params.filter_keypoint_trajectories) {
      computeVelAndAcc(m_tracks.skeletons.at(t_track_idx),
                       m_detections.skeletons.at(t_det_idx),
                       (m_skeleton_groups_buffer.begin()->first - m_track_id_to_time_stamp_map.at(id)).toSec());
    }

    m_track_id_to_time_stamp_map.at(id) = m_skeleton_groups_buffer.begin()->first;

    m_tracks.src_time = m_detections.src_time;
    m_tracks.skeletons.at(t_track_idx) = m_detections.skeletons.at(t_det_idx);
    m_tracks.skeletons.at(t_track_idx).id = id;

    if (m_params.filter_keypoint_trajectories) {
      m_track_id_to_filter_map.at(id).filter(m_tracks.skeletons.at(t_track_idx),
                                             m_track_id_to_time_stamp_map.at(id).toSec());
    }
  }
}

void hiros::track::Tracker::addNewTrack(const hiros::skeletons::types::Skeleton& t_detection)
{
  if (utils::numberOfKeypoints(t_detection) >= m_params.min_keypoints) {
    m_tracks.src_time = m_detections.src_time;
    m_tracks.skeletons.push_back(t_detection);
    m_tracks.skeletons.back().id = ++m_last_track_id;
    initializeVelAndAcc(m_tracks.skeletons.back());

    m_track_id_to_time_stamp_map.emplace(m_last_track_id, m_skeleton_groups_buffer.begin()->first);

    if (m_params.filter_keypoint_trajectories) {
      m_track_id_to_filter_map.emplace(m_tracks.skeletons.back().id,
                                       hiros::track::Filter(m_tracks.skeletons.back(),
                                                            m_track_id_to_time_stamp_map.at(m_last_track_id).toSec(),
                                                            m_params.filter_cutoff_frequency));
    }
  }
}

bool hiros::track::Tracker::unassociatedDetection(const unsigned int& t_det_idx) const
{
  return !m_munkres.colHasMatch(t_det_idx);
}

bool hiros::track::Tracker::unassociatedTrack(const unsigned int& t_track_idx) const
{
  return !m_munkres.rowHasMatch(t_track_idx);
}
