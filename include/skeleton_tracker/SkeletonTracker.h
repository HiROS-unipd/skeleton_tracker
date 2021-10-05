#ifndef hiros_skeleton_tracker_SkeletonTracker_h
#define hiros_skeleton_tracker_SkeletonTracker_h

// ROS dependencies
#include <ros/ros.h>

// OpenCV dependencies
#include "opencv2/opencv.hpp"

// Custom Ros Message dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/types.h"

// Custom Internal dependencies
#include "skeleton_tracker/Filter.h"
#include "skeleton_tracker/Munkres.h"
#include "skeleton_tracker/SkeletonGroupBuffer.h"
#include "skeleton_tracker/utils.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace track {

    struct SkeletonTrackerParameters
    {
      std::string node_name;

      std::vector<std::string> in_skeleton_group_topics;
      std::string out_msg_topic_name;

      ros::Duration fixed_delay;
      int min_markers;
      int min_links;
      double min_linear_distance;
      double max_linear_distance;
      double min_angular_distance;
      double max_angular_distance;
      ros::Duration max_delta_t;
      bool use_positions;
      bool use_linear_velocities;
      bool use_orientations;
      bool use_angular_velocities;
      double velocity_weight;
      bool weight_distances_by_confidences;
      bool weight_distances_by_velocities;
    };

    class SkeletonTracker
    {
    public:
      SkeletonTracker();
      ~SkeletonTracker();

      void configure();
      void start();

    private:
      void stop();
      void setupRosTopics();

      void detectorCallback(const hiros_skeleton_msgs::SkeletonGroup& t_skeleton_group_msg);
      void checkFrameIdConsistency(const hiros_skeleton_msgs::SkeletonGroup& t_skeleton_group_msg);
      void addNewSkeletonGroupToBuffer(const hiros_skeleton_msgs::SkeletonGroup& t_skeleton_group_msg);
      void trackOldestFrame();

      ros::Time getPreviousSrcTime() const;
      ros::Time getCurrentSrcTime() const;
      void publishTracks();

      void fillDetections();
      void createCostMatrix();
      void solveMunkres();
      void removeDistantMatches();
      void updateDetectedTracks();
      void addNewTracks();
      void removeUnassociatedTracks();
      void eraseOldSkeletonGroupFromBuffer();

      double computeWeight(const double& t_det_confidence, const hiros::skeletons::types::Vector3& t_track_vel) const;
      double computeLinPosDistance(const hiros::skeletons::types::KinematicState& t_det_ks,
                                   const hiros::skeletons::types::KinematicState& t_track_ks,
                                   const double& t_weight = 1) const;
      double computeLinVelDistance(const hiros::skeletons::types::KinematicState& t_det_ks,
                                   const hiros::skeletons::types::KinematicState& t_track_ks,
                                   const double& t_weight = 1) const;
      double computeAngPosDistance(const hiros::skeletons::types::KinematicState& t_det_ks,
                                   const hiros::skeletons::types::KinematicState& t_track_ks,
                                   const double& t_weight = 1) const;
      double computeAngVelDistance(const hiros::skeletons::types::KinematicState& t_det_ks,
                                   const hiros::skeletons::types::KinematicState& t_track_ks,
                                   const double& t_weight = 1) const;
      double computeMeanDistance(const std::vector<double>& t_pos_dist,
                                 const std::vector<double>& t_vel_dist = std::vector<double>(),
                                 const bool& t_weighted = false) const;

      double computeMarkersLinDistance(const hiros::skeletons::types::Skeleton& t_track,
                                       const hiros::skeletons::types::Skeleton& t_detection,
                                       const bool& t_weighted = false) const;
      double computeMarkersAngDistance(const hiros::skeletons::types::Skeleton& t_track,
                                       const hiros::skeletons::types::Skeleton& t_detection,
                                       const bool& t_weighted = false) const;

      double computeLinksLinDistance(const hiros::skeletons::types::Skeleton& t_track,
                                     const hiros::skeletons::types::Skeleton& t_detection,
                                     const bool& t_weighted = false) const;
      double computeLinksAngDistance(const hiros::skeletons::types::Skeleton& t_track,
                                     const hiros::skeletons::types::Skeleton& t_detection,
                                     const bool& t_weighted = false) const;

      double computeLinearDistance(const hiros::skeletons::types::Skeleton& t_track,
                                   const hiros::skeletons::types::Skeleton& t_detection,
                                   const bool& t_weighted = false) const;
      double computeAngularDistance(const hiros::skeletons::types::Skeleton& t_track,
                                    const hiros::skeletons::types::Skeleton& t_detection,
                                    const bool& t_weighted = false) const;

      double computeWeightedDistance(const hiros::skeletons::types::Skeleton& t_track,
                                     const hiros::skeletons::types::Skeleton& t_detection) const;

      void initializeVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton) const;
      void initializeVelAndAcc(hiros::skeletons::types::KinematicState& t_state) const;

      void updateDetectedTrack(const unsigned int& t_track_idx, const unsigned int& t_det_idx);
      void addNewTrack(const hiros::skeletons::types::Skeleton& t_detection);
      bool unassociatedDetection(const unsigned int& t_det_idx) const;
      bool unassociatedTrack(const unsigned int& t_track_idx) const;

      ros::NodeHandle m_nh;
      std::string m_node_namespace;

      SkeletonTrackerParameters m_params;

      unsigned long m_n_detectors;
      std::vector<ros::Subscriber> m_in_skeleton_group_subs;
      ros::Publisher m_out_msg_pub;

      // map<src_frame, frame_id>
      std::map<std::string, std::string> m_received_frames;

      SkeletonGroupBuffer m_skeleton_group_buffer;

      Munkres m_munkres;
      cv::Mat_<double> m_linear_distance_matrix;
      cv::Mat_<double> m_angular_distance_matrix;
      cv::Mat_<double> m_cost_matrix;
      cv::Mat_<int> m_munkres_matrix;
      int m_last_track_id;

      hiros::skeletons::types::SkeletonGroup m_detections{};
      hiros::skeletons::types::SkeletonGroup m_tracks{};

      const double k_cutoff_frequency = 10; // [Hz]
      // map<track_id, filter>
      std::map<int, Filter> m_track_filters;

      bool m_configured;
    };
  } // namespace track
} // namespace hiros

#endif
