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
      int min_joints;
      double min_markers_distance;
      double max_markers_distance;
      double min_orientations_distance;
      double max_orientations_distance;
      ros::Duration max_delta_t;
      bool use_markers;
      bool use_marker_velocities;
      bool use_orientations;
      bool use_orientation_velocities;
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

      double computeMarkersDistance(const hiros::skeletons::types::Skeleton& t_track,
                                    const hiros::skeletons::types::Skeleton& t_detection) const;
      double computeOrientationsDistance(const hiros::skeletons::types::Skeleton& t_track,
                                         const hiros::skeletons::types::Skeleton& t_detection) const;
      double computeWeightedMarkersDistance(const hiros::skeletons::types::Skeleton& t_track,
                                            const hiros::skeletons::types::Skeleton& t_detection) const;
      double computeWeightedOrientationsDistance(const hiros::skeletons::types::Skeleton& t_track,
                                                 const hiros::skeletons::types::Skeleton& t_detection) const;
      double computeWeightedDistance(const hiros::skeletons::types::Skeleton& t_track,
                                     const hiros::skeletons::types::Skeleton& t_detection) const;

      void initializeVelAndAcc(hiros::skeletons::types::Skeleton& t_skeleton) const;
      void computeVelAndAcc(const hiros::skeletons::types::Skeleton& t_track,
                            hiros::skeletons::types::Skeleton& t_detection,
                            const double& t_dt) const;
      hiros::skeletons::types::Velocity computeVelocity(const hiros::skeletons::types::Point& t_prev,
                                                        const hiros::skeletons::types::Point& t_curr,
                                                        const double& t_dt) const;
      hiros::skeletons::types::Velocity computeVelocity(const hiros::skeletons::types::MIMU& t_prev,
                                                        const hiros::skeletons::types::MIMU& t_curr,
                                                        const double& t_dt) const;
      hiros::skeletons::types::Acceleration computeAcceleration(const hiros::skeletons::types::Point& t_prev,
                                                                const hiros::skeletons::types::Point& t_curr,
                                                                const double& t_dt) const;
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

      // vector<pair<src_frame, frame_id>>
      std::map<std::string, std::string> m_received_frames;

      SkeletonGroupBuffer m_skeleton_group_buffer;

      Munkres m_munkres;
      cv::Mat_<double> m_markers_distance_matrix;
      cv::Mat_<double> m_orientations_distance_matrix;
      cv::Mat_<double> m_cost_matrix;
      cv::Mat_<int> m_munkres_matrix;
      int m_last_track_id;

      hiros::skeletons::types::SkeletonGroup m_detections{};
      hiros::skeletons::types::SkeletonGroup m_tracks{};

      const double k_cutoff_frequency = 10; // [Hz]
      // map<track_id, filters>
      std::map<int, Filter> m_track_filters;

      bool m_configured;
    };
  } // namespace track
} // namespace hiros

#endif
