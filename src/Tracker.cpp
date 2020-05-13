// ROS dependencies
#include <ros/ros.h>
#include <std_msgs/Header.h>

// Custom Ros Message dependencies
#include "skeleton_msgs/SkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/Munkres.h"
#include "skeleton_tracker/Tracker.h"
#include "skeleton_tracker/utils.h"

hiros::track::Tracker::Tracker()
  : m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_last_track_id(-1)
  , m_configured(false)
{
  m_tracks = std::make_shared<skeletons::types::SkeletonGroup>();
  m_detections = std::make_shared<skeletons::types::SkeletonGroup>();
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
  m_nh.getParam("in_skeleton_group_topic", m_params.in_skeleton_group_topic);
  m_nh.getParam("out_msg_topic_name", m_params.out_msg_topic_name);

  if (m_params.in_skeleton_group_topic.empty() || m_params.out_msg_topic_name.empty()) {
    ROS_FATAL_STREAM("Required topics configuration not provided. Unable to continue");
    ros::shutdown();
  }

  double max_delta_t = 0.;
  m_nh.getParam("min_keypoints", m_params.min_keypoints);
  m_nh.getParam("max_distance", m_params.max_distance);
  m_nh.getParam("max_delta_t", max_delta_t);
  m_params.max_delta_t = ros::Duration(max_delta_t);

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

  if (m_in_skeleton_group_sub) {
    m_in_skeleton_group_sub.shutdown();
  }

  if (m_out_msg_pub) {
    m_out_msg_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Tracker...STOPPED" << BASH_MSG_RESET);
}

std::string hiros::track::Tracker::extractImageQualityFromTopicName(const std::string& t_topic_name) const
{
  auto tmp = t_topic_name.find_last_of("/");
  std::string tmp2 = t_topic_name.substr(0, tmp - 1);
  auto tmp3 = tmp2.find_last_of("/");
  return t_topic_name.substr(tmp3 + 1, tmp - tmp3 - 1);
}

void hiros::track::Tracker::setupRosTopics()
{
  m_in_skeleton_group_sub = m_nh.subscribe(m_params.in_skeleton_group_topic, 1, &Tracker::trackingCallback, this);
  while (m_in_skeleton_group_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_THROTTLE(2, m_node_namespace << " No input messages on skeleton group topic");
    ros::Duration(2).sleep();
  }

  std::string out_msg_topic = m_params.out_msg_topic_name;
  std::string image_quality = extractImageQualityFromTopicName(m_params.in_skeleton_group_topic);
  if (!image_quality.empty()) {
    out_msg_topic.insert(0, image_quality + "/");
  }
  m_out_msg_pub = m_nh.advertise<skeleton_msgs::SkeletonGroup>(out_msg_topic, 1);
}

void hiros::track::Tracker::trackingCallback(const skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  m_skeleton_group_src_time = t_skeleton_group_msg->src_time;
  m_skeleton_group = hiros::skeletons::utils::toStruct(*t_skeleton_group_msg);

  track();

  m_out_msg_pub.publish(skeletons::utils::toMsg(
    ros::Time::now(), t_skeleton_group_msg->header.frame_id, m_skeleton_group_src_time, *m_tracks.get()));
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

void hiros::track::Tracker::fillDetections()
{
  m_detections->skeletons.clear();
  for (auto& skeleton : m_skeleton_group.skeletons) {
    if (!utils::isEmpty(skeleton)) {
      m_detections->skeletons.push_back(skeleton);
    }
  }
}

void hiros::track::Tracker::createCostMatrix()
{
  if (m_tracks->skeletons.empty() || m_detections->skeletons.empty()) {
    m_cost_matrix = cv::Mat_<double>();
    return;
  }

  m_cost_matrix =
    cv::Mat_<double>(static_cast<int>(m_tracks->skeletons.size()), static_cast<int>(m_detections->skeletons.size()));

  for (unsigned int track_idx = 0; track_idx < m_tracks->skeletons.size(); ++track_idx) {
    for (unsigned int det_idx = 0; det_idx < m_detections->skeletons.size(); ++det_idx) {
      m_cost_matrix(static_cast<int>(track_idx), static_cast<int>(det_idx)) =
        computeDistance(m_tracks->skeletons.at(track_idx), m_detections->skeletons.at(det_idx));
    }
  }

  utils::replaceNans(m_cost_matrix);
}

void hiros::track::Tracker::solveMunkres()
{
  utils::Munkres munkres;
  // Munkres: rows = tracks, cols = detections
  m_munkres_matrix = munkres.solve(m_cost_matrix);
}

void hiros::track::Tracker::removeDistantMatches()
{
  if (m_params.max_distance >= 0.) {
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
  for (unsigned int track_idx = 0; track_idx < m_tracks->skeletons.size(); ++track_idx) {
    for (unsigned int det_idx = 0; det_idx < m_detections->skeletons.size(); ++det_idx) {
      updateDetectedTrack(track_idx, det_idx);
    }
  }
}

void hiros::track::Tracker::addNewTracks()
{
  if (m_detections->skeletons.empty()) {
    return;
  }

  if (m_cost_matrix.empty()) {
    for (auto& detection : m_detections->skeletons) {
      addNewTrack(detection);
    }
  }
  else {
    for (unsigned int c = 0; c < static_cast<unsigned int>(m_munkres_matrix.cols); ++c) {
      if (unassociatedDetection(c)) {
        addNewTrack(m_detections->skeletons.at(c));
      }
    }
  }
}

void hiros::track::Tracker::removeUnassociatedTracks()
{
  ros::Duration delta_t;

  for (int track_idx = 0, index_to_erase = 0; track_idx < m_munkres_matrix.rows; ++track_idx, ++index_to_erase) {
    if (unassociatedTrack(static_cast<unsigned int>(track_idx))) {
      delta_t = m_skeleton_group_src_time
                - m_track_id_to_time_stamp_map.at(m_tracks->skeletons.at(static_cast<unsigned int>(index_to_erase)).id);

      if (delta_t > m_params.max_delta_t) {
        m_track_id_to_time_stamp_map.erase(m_tracks->skeletons.at(static_cast<unsigned int>(index_to_erase)).id);
        m_tracks->skeletons.erase(m_tracks->skeletons.begin() + index_to_erase--);
      }
    }
  }
}

double hiros::track::Tracker::computeDistance(const hiros::skeletons::types::Skeleton& t_track,
                                              const hiros::skeletons::types::Skeleton& t_detection) const
{
  double dist = 0.;
  unsigned int n_kps = 0;
  for (auto& det_kpg : t_detection.skeleton_parts) {
    for (auto& det_kp : det_kpg.keypoints) {
      hiros::skeletons::types::Keypoint track_kp = utils::findKeypoint(t_track, det_kpg.id, det_kp.id);

      if (!std::isnan(track_kp.point.x) && !std::isnan(track_kp.point.y)) {
        dist += utils::distance(det_kp.point, track_kp.point);
        ++n_kps;
      }
    }
  }

  // Multiplying by 1/n_kps^0.5 to prefer track-detection matching when the number of detected keypoints is higher
  return dist = (n_kps == 0) ? std::numeric_limits<double>::quiet_NaN() : dist / std::pow(n_kps, 1.5);
}

void hiros::track::Tracker::updateDetectedTrack(const unsigned int& t_track_idx, const unsigned int& t_det_idx)
{
  if (match(t_track_idx, t_det_idx)) {
    int track_id = m_tracks->skeletons.at(t_track_idx).id;
    m_track_id_to_time_stamp_map.at(track_id) = m_skeleton_group_src_time;
    m_tracks->skeletons.at(t_track_idx) = m_detections->skeletons.at(t_det_idx);
    m_tracks->skeletons.at(t_track_idx).id = track_id;
  }
}

void hiros::track::Tracker::addNewTrack(const hiros::skeletons::types::Skeleton& t_detection)
{
  if (utils::numberOfKeypoints(t_detection) >= m_params.min_keypoints) {
    m_tracks->skeletons.push_back(t_detection);
    m_tracks->skeletons.back().id = ++m_last_track_id;
    m_track_id_to_time_stamp_map.emplace(m_last_track_id, m_skeleton_group_src_time);
  }
}

bool hiros::track::Tracker::unassociatedDetection(const unsigned int& t_det_idx) const
{
  return !colHasMatch(t_det_idx);
}

bool hiros::track::Tracker::unassociatedTrack(const unsigned int& t_track_idx) const
{
  return !rowHasMatch(t_track_idx);
}

bool hiros::track::Tracker::match(const unsigned int& t_track_idx, const unsigned int& t_det_idx) const
{
  return (m_munkres_matrix(static_cast<int>(t_track_idx), static_cast<int>(t_det_idx)) == 1);
}

bool hiros::track::Tracker::colHasMatch(const unsigned int& t_col) const
{
  for (int r = 0; r < m_munkres_matrix.rows; ++r) {
    if (m_munkres_matrix(r, static_cast<int>(t_col)) == 1) {
      return true;
    }
  }

  return false;
}

int hiros::track::Tracker::findMatchInCol(const unsigned int& t_col) const
{
  for (int r = 0; r < m_munkres_matrix.rows; ++r) {
    if (m_munkres_matrix(r, static_cast<int>(t_col)) == 1) {
      return r;
    }
  }

  return -1;
}

bool hiros::track::Tracker::rowHasMatch(const unsigned int& t_row) const
{
  for (int c = 0; c < m_munkres_matrix.cols; ++c) {
    if (m_munkres_matrix(static_cast<int>(t_row), c) == 1) {
      return true;
    }
  }

  return false;
}

int hiros::track::Tracker::findMatchInRow(const unsigned int& t_row) const
{
  for (int c = 0; c < m_munkres_matrix.cols; ++c) {
    if (m_munkres_matrix(static_cast<int>(t_row), c) == 1) {
      return c;
    }
  }

  return -1;
}
