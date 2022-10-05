#ifndef hiros_skeleton_tracker_Tracker_h
#define hiros_skeleton_tracker_Tracker_h

// ROS dependencies
#include <rclcpp/rclcpp.hpp>

// OpenCV dependencies
#include "opencv2/opencv.hpp"

// Custom ROS message dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"

// Custom external packages dependencies
#include "skeleton_filter/SkeletonFilter.h"
#include "skeletons/types.h"

// Custom internal dependencies
#include "skeleton_tracker/Munkres.h"
#include "skeleton_tracker/SkeletonGroupBuffer.h"
#include "skeleton_tracker/utils.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
namespace skeletons {

class Tracker : public rclcpp::Node {
 public:
  Tracker();
  ~Tracker();

 private:
  struct Parameters {
    std::vector<std::string> input_topics{};
    std::string output_topic{};

    rclcpp::Duration fixed_delay{std::chrono::milliseconds()};
    double min_skeleton_confidence{};
    double min_marker_confidence{};
    double min_link_confidence{};
    int min_markers{};
    int min_links{};
    double min_linear_distance{};
    double max_linear_distance{};
    double min_angular_distance{};
    double max_angular_distance{};
    rclcpp::Duration max_delta_t{std::chrono::milliseconds()};
    bool use_positions{};
    bool use_linear_velocities{};
    bool use_orientations{};
    bool use_angular_velocities{};
    double velocity_weight{};
    bool weight_distances_by_confidences{};
    bool weight_distances_by_velocities{};
  };

  template <typename T>
  bool getParam(const std::string& name, T& parameter) {
    declare_parameter<T>(name);
    return get_parameter(name, parameter);
  }

  void start();
  void stop() const;
  void configure();

  void getParams();
  void setupRosTopics();

  void checkFrameIdConsistency(
      const hiros_skeleton_msgs::msg::SkeletonGroup& msg);
  void addNewSkeletonGroupToBuffer(
      const hiros_skeleton_msgs::msg::SkeletonGroup& msg);
  void trackOldestFrame();
  void publishTracks();

  rclcpp::Time getPreviousSrcTime() const;
  rclcpp::Time getCurrentSrcTime() const;
  hiros_skeleton_msgs::msg::SkeletonGroup filterByConfidence(
      const hiros_skeleton_msgs::msg::SkeletonGroup& msg) const;

  void fillDetections();
  void createCostMatrix();
  void solveMunkres();
  void removeDistantMatches();
  void updateDetectedTracks();
  void addNewTracks();
  void removeUnassociatedTracks();
  void eraseOldSkeletonGroupFromBuffer();

  double computeWeight(const double& det_confidence,
                       const hiros::skeletons::types::Vector3& track_vel) const;
  double computeLinPosDistance(
      const hiros::skeletons::types::KinematicState& det_ks,
      const hiros::skeletons::types::KinematicState& track_ks,
      const double& weight = 1) const;
  double computeLinVelDistance(
      const hiros::skeletons::types::KinematicState& det_ks,
      const hiros::skeletons::types::KinematicState& track_ks,
      const double& weight = 1) const;
  double computeAngPosDistance(
      const hiros::skeletons::types::KinematicState& det_ks,
      const hiros::skeletons::types::KinematicState& track_ks,
      const double& weight = 1) const;
  double computeAngVelDistance(
      const hiros::skeletons::types::KinematicState& det_ks,
      const hiros::skeletons::types::KinematicState& track_ks,
      const double& weight = 1) const;
  double computeMeanDistance(
      const std::vector<double>& pos_dists,
      const std::vector<double>& vel_dists = std::vector<double>()) const;

  double computeMarkersLinDistance(
      const hiros::skeletons::types::Skeleton& track,
      const hiros::skeletons::types::Skeleton& detection,
      const bool& weighted = false) const;
  double computeMarkersAngDistance(
      const hiros::skeletons::types::Skeleton& track,
      const hiros::skeletons::types::Skeleton& detection,
      const bool& weighted = false) const;

  double computeLinksLinDistance(
      const hiros::skeletons::types::Skeleton& track,
      const hiros::skeletons::types::Skeleton& detection,
      const bool& weighted = false) const;
  double computeLinksAngDistance(
      const hiros::skeletons::types::Skeleton& track,
      const hiros::skeletons::types::Skeleton& detection,
      const bool& weighted = false) const;

  double computeLinearDistance(
      const hiros::skeletons::types::Skeleton& track,
      const hiros::skeletons::types::Skeleton& detection,
      const bool& weighted = false) const;
  double computeAngularDistance(
      const hiros::skeletons::types::Skeleton& track,
      const hiros::skeletons::types::Skeleton& detection,
      const bool& weighted = false) const;

  double computeWeightedDistance(const hiros::skeletons::types::Skeleton& track,
                                 hiros::skeletons::types::Skeleton detection);

  void initializeVelAndAcc(hiros::skeletons::types::Skeleton& skeleton) const;
  void initializeVelAndAcc(
      hiros::skeletons::types::KinematicState& state) const;
  void updVelAndAcc(hiros::skeletons::SkeletonFilter& filter,
                    hiros::skeletons::types::Skeleton& skeleton) const;

  void updateDetectedTrack(const unsigned int& track_idx,
                           const unsigned int& det_idx);
  void addNewTrack(const hiros::skeletons::types::Skeleton& detection);
  bool unassociatedDetection(const unsigned int& det_idx) const;
  bool unassociatedTrack(const unsigned int& track_idx) const;

  void callback(const hiros_skeleton_msgs::msg::SkeletonGroup& msg);

  Parameters params_{};

  unsigned long n_detectors{};
  std::vector<
      rclcpp::Subscription<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr>
      subs_{};
  rclcpp::Publisher<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr pub_{};

  // map<src_frame, frame_id>
  std::map<std::string, std::string> received_frames_{};

  SkeletonGroupBuffer skeleton_group_buffer_{};

  Munkres munkres_{};
  cv::Mat_<double> linear_distance_matrix_{};
  cv::Mat_<double> angular_distance_matrix_{};
  cv::Mat_<double> cost_matrix_{};
  cv::Mat_<int> munkres_matrix_{};
  int last_track_id_{-1};

  hiros::skeletons::types::SkeletonGroup detections_{};
  hiros::skeletons::types::SkeletonGroup tracks_{};

  const double k_cutoff_frequency{10};  // [Hz]
  // map<track_id, filter>
  std::map<int, hiros::skeletons::SkeletonFilter> track_filters_{};
};

}  // namespace skeletons
}  // namespace hiros

#endif
