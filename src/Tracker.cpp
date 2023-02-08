// Standard dependencies
#include <numeric>

// Custom External Packages dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_tracker/Tracker.h"
#include "skeleton_tracker/utils.h"

hiros::skeletons::Tracker::Tracker() : Node("hiros_skeleton_tracker") {
  start();
}

hiros::skeletons::Tracker::~Tracker() { stop(); }

void hiros::skeletons::Tracker::start() {
  configure();

  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Running" << BASH_MSG_RESET);
}

void hiros::skeletons::Tracker::stop() const {
  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Stopped" << BASH_MSG_RESET);

  rclcpp::shutdown();
}

void hiros::skeletons::Tracker::configure() {
  getParams();
  setupRosTopics();
}

void hiros::skeletons::Tracker::getParams() {
  getParam("input_topics", params_.input_topics);
  n_detectors = params_.input_topics.size();

  getParam("output_topic", params_.output_topic);
  auto fixed_delay{0.};
  getParam("fixed_delay", fixed_delay);
  params_.fixed_delay = rclcpp::Duration::from_seconds(fixed_delay);

  getParam("min_skeleton_confidence", params_.min_skeleton_confidence);
  getParam("min_marker_confidence", params_.min_marker_confidence);
  getParam("min_link_confidence", params_.min_link_confidence);
  getParam("min_markers", params_.min_markers);
  getParam("min_links", params_.min_links);
  getParam("min_linear_distance", params_.min_linear_distance);
  if (params_.min_linear_distance <= 0) {
    params_.min_linear_distance = std::numeric_limits<double>::max();
  }

  getParam("max_linear_distance", params_.max_linear_distance);
  if (params_.max_linear_distance <= 0) {
    params_.max_linear_distance = std::numeric_limits<double>::max();
  }

  getParam("min_angular_distance", params_.min_angular_distance);
  if (params_.min_angular_distance <= 0) {
    params_.min_angular_distance = std::numeric_limits<double>::max();
  }

  getParam("max_angular_distance", params_.max_angular_distance);
  if (params_.max_angular_distance <= 0) {
    params_.max_angular_distance = std::numeric_limits<double>::max();
  }

  auto max_delta_t{0.};
  getParam("max_delta_t", max_delta_t);
  params_.max_delta_t = rclcpp::Duration::from_seconds(max_delta_t);

  getParam("use_positions", params_.use_positions);
  getParam("use_linear_velocities", params_.use_linear_velocities);
  getParam("use_orientations", params_.use_orientations);
  getParam("use_angular_velocities", params_.use_angular_velocities);
  getParam("velocity_weight", params_.velocity_weight);
  getParam("weight_distances_by_confidences",
           params_.weight_distances_by_confidences);
  getParam("weight_distances_by_velocities",
           params_.weight_distances_by_velocities);
}

void hiros::skeletons::Tracker::setupRosTopics() {
  for (const auto& topic : params_.input_topics) {
    subs_.push_back(
        create_subscription<hiros_skeleton_msgs::msg::SkeletonGroup>(
            topic, 10,
            std::bind(&Tracker::callback, this, std::placeholders::_1)));
  }

  pub_ = create_publisher<hiros_skeleton_msgs::msg::SkeletonGroup>(
      params_.output_topic, 10);
}

void hiros::skeletons::Tracker::checkFrameIdConsistency(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  if (received_frames_.size() < n_detectors) {
    received_frames_[msg.skeletons.front().src_frame] = msg.header.frame_id;
    tracks_.frame = msg.header.frame_id;

    if (std::find_if(received_frames_.begin(), received_frames_.end(),
                     [&](const auto& e) {
                       return e.second != tracks_.frame;
                     }) != received_frames_.end()) {
      RCLCPP_FATAL_STREAM(
          get_logger(),
          "Data must be expressed w.r.t the same reference frame");
      stop();
      exit(EXIT_FAILURE);
    }
  }
}

void hiros::skeletons::Tracker::addNewSkeletonGroupToBuffer(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  skeleton_group_buffer_.push_back(filterByConfidence(msg));
}

void hiros::skeletons::Tracker::trackOldestFrame() {
  fillDetections();
  createCostMatrix();
  solveMunkres();
  removeDistantMatches();
  updateDetectedTracks();
  addNewTracks();
  removeUnassociatedTracks();
}

void hiros::skeletons::Tracker::publishTracks() {
  tracks_.time = now().seconds();
  pub_->publish(skeletons::utils::toMsg(tracks_));
}

rclcpp::Time hiros::skeletons::Tracker::getPreviousSrcTime() const {
  // Return the source time of the latest available tracks
  return utils::newestSrcTime(tracks_);
}

rclcpp::Time hiros::skeletons::Tracker::getCurrentSrcTime() const {
  // If m_detections is already filled, then the current src_time is the max
  // among the detections' src_times
  if (!detections_.skeletons.empty()) {
    return utils::newestSrcTime(detections_);
  }
  // Else, if the skeleton_group buffer is not empty, the current src_time is
  // the one from the oldest frame in the buffer
  else if (!skeleton_group_buffer_.empty()) {
    return skeleton_group_buffer_.get_src_time();
  }

  return rclcpp::Time();
}

hiros_skeleton_msgs::msg::SkeletonGroup
hiros::skeletons::Tracker::filterByConfidence(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) const {
  hiros_skeleton_msgs::msg::SkeletonGroup filtered_msg{msg};

  // Remove skeletons with confidence < min_skeleton_confidence
  filtered_msg.skeletons.erase(
      std::remove_if(
          filtered_msg.skeletons.begin(), filtered_msg.skeletons.end(),
          [&](auto& skel) {
            return (skel.confidence >= 0 &&
                    skel.confidence < params_.min_skeleton_confidence);
          }),
      filtered_msg.skeletons.end());

  // Remove markers with confidence < min_marker_confidence
  for (auto& skel : filtered_msg.skeletons) {
    skel.markers.erase(
        std::remove_if(skel.markers.begin(), skel.markers.end(),
                       [&](const auto& mk) {
                         return (mk.confidence >= 0 &&
                                 mk.confidence < params_.min_marker_confidence);
                       }),
        skel.markers.end());
  }

  // Remove links with confidence < min_link_confidence
  for (auto& skel : filtered_msg.skeletons) {
    skel.links.erase(
        std::remove_if(skel.links.begin(), skel.links.end(),
                       [&](const auto& lk) {
                         return (lk.confidence >= 0 &&
                                 lk.confidence < params_.min_link_confidence);
                       }),
        skel.links.end());
  }

  return filtered_msg;
}

void hiros::skeletons::Tracker::fillDetections() {
  detections_.skeletons.clear();

  for (const auto& skeleton :
       skeleton_group_buffer_.get_skeleton_group().skeletons) {
    if (!utils::isEmpty(skeleton)) {
      detections_.addSkeleton(hiros::skeletons::utils::toStruct(skeleton));
    }
  }

  eraseOldSkeletonGroupFromBuffer();
}

void hiros::skeletons::Tracker::createCostMatrix() {
  if (tracks_.skeletons.empty() || detections_.skeletons.empty()) {
    cost_matrix_ = cv::Mat_<double>();
    linear_distance_matrix_ = cv::Mat_<double>();
    angular_distance_matrix_ = cv::Mat_<double>();
    return;
  }

  cost_matrix_ =
      cv::Mat_<double>(static_cast<int>(tracks_.skeletons.size()),
                       static_cast<int>(detections_.skeletons.size()));
  linear_distance_matrix_ =
      cv::Mat_<double>(static_cast<int>(tracks_.skeletons.size()),
                       static_cast<int>(detections_.skeletons.size()));
  angular_distance_matrix_ =
      cv::Mat_<double>(static_cast<int>(tracks_.skeletons.size()),
                       static_cast<int>(detections_.skeletons.size()));

  for (auto track_idx{0}; track_idx < cost_matrix_.rows; ++track_idx) {
    for (auto det_idx{0}; det_idx < cost_matrix_.cols; ++det_idx) {
      auto predicted_track{utils::predict(
          tracks_.skeletons.at(static_cast<unsigned int>(track_idx)),
          detections_.skeletons.at(static_cast<unsigned int>(det_idx))
              .src_time)};

      linear_distance_matrix_(track_idx, det_idx) = computeLinearDistance(
          predicted_track,
          detections_.skeletons.at(static_cast<unsigned int>(det_idx)));
      angular_distance_matrix_(track_idx, det_idx) = computeAngularDistance(
          predicted_track,
          detections_.skeletons.at(static_cast<unsigned int>(det_idx)));
      cost_matrix_(track_idx, det_idx) = computeWeightedDistance(
          predicted_track,
          detections_.skeletons.at(static_cast<unsigned int>(det_idx)));
    }
  }
}

void hiros::skeletons::Tracker::solveMunkres() {
  // Munkres: rows = tracks, cols = detections
  munkres_matrix_ = munkres_.solve(cost_matrix_);
}

void hiros::skeletons::Tracker::removeDistantMatches() {
  for (auto r{0}; r < munkres_matrix_.rows; ++r) {
    for (auto c{0}; c < munkres_matrix_.cols; ++c) {
      if (munkres_matrix_(r, c) == 1 &&
          (linear_distance_matrix_[r][c] > params_.max_linear_distance ||
           angular_distance_matrix_[r][c] > params_.max_angular_distance)) {
        munkres_matrix_(r, c) = 0;
      }
    }
  }
}

void hiros::skeletons::Tracker::updateDetectedTracks() {
  for (auto track_idx{0u}; track_idx < tracks_.skeletons.size(); ++track_idx) {
    for (auto det_idx{0u}; det_idx < detections_.skeletons.size(); ++det_idx) {
      updateDetectedTrack(track_idx, det_idx);
    }
  }
}

void hiros::skeletons::Tracker::addNewTracks() {
  if (detections_.skeletons.empty()) {
    return;
  }

  if (cost_matrix_.empty()) {
    for (const auto& detection : detections_.skeletons) {
      addNewTrack(detection);
    }
  } else {
    for (auto c{0u}; c < static_cast<unsigned int>(munkres_matrix_.cols); ++c) {
      if (unassociatedDetection(c)) {
        double min{};
        unsigned int row{}, col{};
        utils::minWithIndex(cost_matrix_.col(static_cast<int>(c)), min, row,
                            col);

        // If the distance between the unassociated detection and the track is
        // lower than params.min_distance, then consider the detection as part
        // of the track
        if (utils::min(linear_distance_matrix_.col(static_cast<int>(c))) <
                params_.min_linear_distance &&
            utils::min(angular_distance_matrix_.col(static_cast<int>(c))) <
                params_.min_angular_distance) {
          utils::merge(tracks_.skeletons.at(row), detections_.skeletons.at(c));
        } else {
          addNewTrack(detections_.skeletons.at(c));
        }
      }
    }
  }
}

void hiros::skeletons::Tracker::removeUnassociatedTracks() {
  unsigned int track_idx{};
  int id{};
  rclcpp::Duration delta_t{std::chrono::milliseconds()};

  for (auto i{static_cast<int>(tracks_.skeletons.size()) - 1}; i >= 0; --i) {
    track_idx = static_cast<unsigned int>(i);

    if (unassociatedTrack(track_idx)) {
      id = tracks_.skeletons.at(track_idx).id;
      delta_t = getCurrentSrcTime() -
                rclcpp::Time{static_cast<long>(
                    tracks_.skeletons.at(track_idx).src_time * 1e9)};

      if (delta_t > params_.max_delta_t) {
        tracks_.removeSkeleton(id);
        track_filters_.erase(id);
      }
    }
  }
}

void hiros::skeletons::Tracker::eraseOldSkeletonGroupFromBuffer() {
  skeleton_group_buffer_.pop_front();
}

double hiros::skeletons::Tracker::computeWeight(
    const double& det_confidence,
    const hiros::skeletons::types::Vector3& track_vel) const {
  auto weight{(params_.weight_distances_by_confidences &&
               !std::isnan(det_confidence) && det_confidence >= 0)
                  ? det_confidence
                  : 1};
  if (params_.weight_distances_by_velocities) {
    // The higher the velocity the lower the marker/link will be weighted. Raise
    // velocity magnitude to the power of -0.25 to have smoother weights: vel =
    // 0 -> weight = 1, vel = 10 -> weight = 0.55, vel = 100 -> weight = 0.32,
    // vel = inf -> weight = 0
    auto vel_weight{
        std::pow(1 + hiros::skeletons::utils::magnitude(track_vel), -0.25)};
    weight *= !std::isnan(vel_weight) ? vel_weight : 1;
  }

  return weight;
}

double hiros::skeletons::Tracker::computeLinPosDistance(
    const hiros::skeletons::types::KinematicState& det_ks,
    const hiros::skeletons::types::KinematicState& track_ks,
    const double& weight) const {
  return params_.use_positions
             ? weight * hiros::skeletons::utils::distance(
                            det_ks.pose.position, track_ks.pose.position)
             : std::numeric_limits<double>::quiet_NaN();
}

double hiros::skeletons::Tracker::computeLinVelDistance(
    const hiros::skeletons::types::KinematicState& det_ks,
    const hiros::skeletons::types::KinematicState& track_ks,
    const double& weight) const {
  return params_.use_linear_velocities
             ? weight * hiros::skeletons::utils::distance(
                            det_ks.velocity.linear, track_ks.velocity.linear)
             : std::numeric_limits<double>::quiet_NaN();
}

double hiros::skeletons::Tracker::computeAngPosDistance(
    const hiros::skeletons::types::KinematicState& det_ks,
    const hiros::skeletons::types::KinematicState& track_ks,
    const double& weight) const {
  return params_.use_orientations
             ? weight * hiros::skeletons::utils::distance(
                            det_ks.pose.orientation, track_ks.pose.orientation)
             : std::numeric_limits<double>::quiet_NaN();
}

double hiros::skeletons::Tracker::computeAngVelDistance(
    const hiros::skeletons::types::KinematicState& det_ks,
    const hiros::skeletons::types::KinematicState& track_ks,
    const double& weight) const {
  return params_.use_angular_velocities
             ? weight * hiros::skeletons::utils::distance(
                            det_ks.velocity.angular, track_ks.velocity.angular)
             : std::numeric_limits<double>::quiet_NaN();
}

double hiros::skeletons::Tracker::computeMeanDistance(
    const std::vector<double>& pos_dists,
    const std::vector<double>& vel_dists) const {
  auto n_pos{std::count_if(pos_dists.begin(), pos_dists.end(),
                           [](const auto& e) { return !std::isnan(e); })};
  auto n_vel{std::count_if(vel_dists.begin(), vel_dists.end(),
                           [](const auto& e) { return !std::isnan(e); })};

  auto pos_dist{std::numeric_limits<double>::quiet_NaN()};
  auto vel_dist{std::numeric_limits<double>::quiet_NaN()};

  if (n_pos > 0) {
    pos_dist = std::accumulate(pos_dists.begin(), pos_dists.end(), 0.0,
                               [&](double a, double b) {
                                 return a + (std::isnan(b) ? 0. : b / n_pos);
                               });

    if (std::isnan(vel_dist)) {
      vel_dist = 0;
    }
  }

  if (n_vel > 0) {
    vel_dist = std::accumulate(pos_dists.begin(), pos_dists.end(), 0.0,
                               [&](double a, double b) {
                                 return a + (std::isnan(b) ? 0. : b / n_vel);
                               });

    if (std::isnan(pos_dist)) {
      vel_dist = 0;
    }
  }

  return pos_dist + params_.velocity_weight * vel_dist;
}

double hiros::skeletons::Tracker::computeMarkersLinDistance(
    const hiros::skeletons::types::Skeleton& track,
    const hiros::skeletons::types::Skeleton& detection,
    const bool& weighted) const {
  // If positions and linear velocities do not have to be used, then the linear
  // distance is 0
  if (!params_.use_positions && !params_.use_linear_velocities) {
    return 0;
  }

  std::vector<double> pos_dists{};
  std::vector<double> vel_dists{};

  for (const auto& det_mk : detection.markers) {
    if (track.hasMarker(det_mk.id)) {
      auto& track_mk{track.getMarker(det_mk.id)};

      auto weight{weighted ? computeWeight(det_mk.confidence,
                                           track_mk.center.velocity.linear)
                           : 1.};
      pos_dists.push_back(
          computeLinPosDistance(det_mk.center, track_mk.center, weight));
      vel_dists.push_back(
          computeLinVelDistance(det_mk.center, track_mk.center, weight));
    }
  }

  return computeMeanDistance(pos_dists, vel_dists);
}

double hiros::skeletons::Tracker::computeMarkersAngDistance(
    const hiros::skeletons::types::Skeleton& track,
    const hiros::skeletons::types::Skeleton& detection,
    const bool& weighted) const {
  // If orientations and angular velocities do not have to be used, then the
  // angular distance is 0
  if (!params_.use_orientations && !params_.use_angular_velocities) {
    return 0;
  }

  std::vector<double> or_dists{};
  std::vector<double> vel_dists{};

  for (const auto& det_mk : detection.markers) {
    if (track.hasMarker(det_mk.id)) {
      auto& track_mk{track.getMarker(det_mk.id)};

      auto weight{weighted ? computeWeight(det_mk.confidence,
                                           track_mk.center.velocity.angular)
                           : 1.};
      or_dists.push_back(
          computeAngPosDistance(det_mk.center, track_mk.center, weight));
      vel_dists.push_back(
          computeAngVelDistance(det_mk.center, track_mk.center, weight));
    }
  }

  return computeMeanDistance(or_dists, vel_dists);
}

double hiros::skeletons::Tracker::computeLinksLinDistance(
    const hiros::skeletons::types::Skeleton& track,
    const hiros::skeletons::types::Skeleton& detection,
    const bool& weighted) const {
  // If positions and linear velocities do not have to be used, then the linear
  // distance is 0
  if (!params_.use_positions && !params_.use_linear_velocities) {
    return 0;
  }

  std::vector<double> pos_dists{};
  std::vector<double> vel_dists{};

  for (const auto& det_lk : detection.links) {
    if (track.hasLink(det_lk.id)) {
      auto& track_lk{track.getLink(det_lk.id)};

      auto weight{weighted ? computeWeight(det_lk.confidence,
                                           track_lk.center.velocity.linear)
                           : 1.};
      pos_dists.push_back(
          computeLinPosDistance(det_lk.center, track_lk.center, weight));
      vel_dists.push_back(
          computeLinVelDistance(det_lk.center, track_lk.center, weight));
    }
  }

  return computeMeanDistance(pos_dists, vel_dists);
}

double hiros::skeletons::Tracker::computeLinksAngDistance(
    const hiros::skeletons::types::Skeleton& track,
    const hiros::skeletons::types::Skeleton& detection,
    const bool& weighted) const {
  // If orientations and angular velocities do not have to be used, then the
  // angular distance is 0
  if (!params_.use_orientations && !params_.use_angular_velocities) {
    return 0;
  }

  std::vector<double> or_dists{};
  std::vector<double> vel_dists{};

  for (const auto& det_lk : detection.links) {
    if (track.hasLink(det_lk.id)) {
      auto& track_lk{track.getLink(det_lk.id)};

      auto weight{weighted ? computeWeight(det_lk.confidence,
                                           track_lk.center.velocity.angular)
                           : 1.};
      or_dists.push_back(
          computeAngPosDistance(det_lk.center, track_lk.center, weight));
      vel_dists.push_back(
          computeAngVelDistance(det_lk.center, track_lk.center, weight));
    }
  }

  return computeMeanDistance(or_dists, vel_dists);
}

double hiros::skeletons::Tracker::computeLinearDistance(
    const hiros::skeletons::types::Skeleton& track,
    const hiros::skeletons::types::Skeleton& detection,
    const bool& weighted) const {
  // If positions and linear velocities do not have to be used, then the linear
  // distance is 0
  if (!params_.use_positions && !params_.use_linear_velocities) {
    return 0;
  }

  auto marker_dist{computeMarkersLinDistance(track, detection, weighted)};
  auto link_dist{computeLinksLinDistance(track, detection, weighted)};

  if (std::isnan(marker_dist) && std::isnan(link_dist)) {
    return std::numeric_limits<double>::max() - 1;
  }

  if (std::isnan(marker_dist)) {
    marker_dist = 0;
  }

  if (std::isnan(link_dist)) {
    link_dist = 0;
  }

  return (marker_dist + link_dist) / 2.;
}

double hiros::skeletons::Tracker::computeAngularDistance(
    const hiros::skeletons::types::Skeleton& track,
    const hiros::skeletons::types::Skeleton& detection,
    const bool& weighted) const {
  // If orientations and angular velocities do not have to be used, then the
  // angular distance is 0
  if (!params_.use_orientations && !params_.use_angular_velocities) {
    return 0;
  }

  auto marker_dist{computeMarkersAngDistance(track, detection, weighted)};
  auto link_dist{computeLinksAngDistance(track, detection, weighted)};

  if (std::isnan(marker_dist) && std::isnan(link_dist)) {
    return std::numeric_limits<double>::max() - 1;
  }

  if (std::isnan(marker_dist)) {
    marker_dist = 0;
  }

  if (std::isnan(link_dist)) {
    link_dist = 0;
  }

  return (marker_dist + link_dist) / 2.;
}

double hiros::skeletons::Tracker::computeWeightedDistance(
    const hiros::skeletons::types::Skeleton& track,
    hiros::skeletons::types::Skeleton detection) {
  updVelAndAcc(track_filters_[track.id], detection);

  return computeLinearDistance(track, detection, true) +
         computeAngularDistance(track, detection, true);
}

void hiros::skeletons::Tracker::initializeVelAndAcc(
    hiros::skeletons::types::Skeleton& skeleton) const {
  for (auto& mk : skeleton.markers) {
    initializeVelAndAcc(mk.center);
  }

  for (auto& lk : skeleton.links) {
    initializeVelAndAcc(lk.center);
  }
}

void hiros::skeletons::Tracker::initializeVelAndAcc(
    hiros::skeletons::types::KinematicState& state) const {
  if (hiros::skeletons::utils::isNaN(state.velocity.linear)) {
    state.velocity.linear = hiros::skeletons::types::Vector3(0, 0, 0);
  }
  if (hiros::skeletons::utils::isNaN(state.velocity.angular)) {
    state.velocity.angular = hiros::skeletons::types::Vector3(0, 0, 0);
  }

  if (hiros::skeletons::utils::isNaN(state.acceleration.linear)) {
    state.acceleration.linear = hiros::skeletons::types::Vector3(0, 0, 0);
  }
  if (hiros::skeletons::utils::isNaN(state.acceleration.angular)) {
    state.acceleration.angular = hiros::skeletons::types::Vector3(0, 0, 0);
  }
}

void hiros::skeletons::Tracker::updVelAndAcc(
    hiros::skeletons::SkeletonFilter& filter,
    hiros::skeletons::types::Skeleton& skeleton) const {
  auto original_skel{skeleton};
  filter.filter(skeleton);

  // Reset marker poses to the values before filtering
  for (auto& mk : skeleton.markers) {
    mk.center.pose = original_skel.getMarker(mk.id).center.pose;
  }

  // Reset link poses to the values before filtering
  for (auto& lk : skeleton.links) {
    lk.center.pose = original_skel.getLink(lk.id).center.pose;
  }
}

void hiros::skeletons::Tracker::updateDetectedTrack(
    const unsigned int& track_idx, const unsigned int& det_idx) {
  if (utils::matchMunkres(munkres_matrix_, track_idx, det_idx)) {
    auto id{tracks_.skeletons.at(track_idx).id};

    tracks_.skeletons.at(track_idx) = detections_.skeletons.at(det_idx);
    tracks_.skeletons.at(track_idx).id = id;
    // Initialize velocity and acceleration for new markers/links
    initializeVelAndAcc(tracks_.skeletons.at(track_idx));
    updVelAndAcc(track_filters_[id], tracks_.skeletons.at(track_idx));
  }
}

void hiros::skeletons::Tracker::addNewTrack(
    const hiros::skeletons::types::Skeleton& detection) {
  if (detection.markers.size() >=
          static_cast<unsigned int>(params_.min_markers) &&
      detection.links.size() >= static_cast<unsigned int>(params_.min_links)) {
    auto new_track{detection};
    new_track.id = ++last_track_id_;
    initializeVelAndAcc(new_track);
    track_filters_[new_track.id] =
        SkeletonFilter(new_track, FilterType::StateSpace, k_cutoff_frequency);
    tracks_.addSkeleton(new_track);
  }
}

bool hiros::skeletons::Tracker::unassociatedDetection(
    const unsigned int& det_idx) const {
  return !munkres_.colHasMatch(det_idx);
}

bool hiros::skeletons::Tracker::unassociatedTrack(
    const unsigned int& track_idx) const {
  return !munkres_.rowHasMatch(track_idx);
}

void hiros::skeletons::Tracker::callback(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  if (msg.skeletons.empty() ||
      utils::newestSrcTime(msg) <= getPreviousSrcTime()) {
    return;
  }

  checkFrameIdConsistency(msg);
  addNewSkeletonGroupToBuffer(msg);

  while (!skeleton_group_buffer_.empty() &&
         (utils::oldestSrcTime(msg) - skeleton_group_buffer_.get_src_time()) >=
             params_.fixed_delay) {
    trackOldestFrame();
    publishTracks();
  }
}
