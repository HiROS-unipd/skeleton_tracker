#ifndef hiros_skeleton_tracker_OrientationSkeletonTracker_h
#define hiros_skeleton_tracker_OrientationSkeletonTracker_h

// ROS dependencies
#include <ros/ros.h>

// OpenCV dependencies
#include "opencv2/opencv.hpp"

// Custom Ros Message dependencies
#include "hiros_skeleton_msgs/OrientationSkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/types.h"

// Custom Internal dependencies
#include "skeleton_tracker/Munkres.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace track {

    struct OrientationSkeletonTrackerParameters
    {
      std::string node_name;

      std::vector<std::string> in_skeleton_group_topics;
      std::string out_msg_topic_name;

      double fixed_delay;
      int min_orientations;
      double min_distance;
      double max_distance;
      ros::Duration max_delta_t;
      bool use_orientations;
      bool use_velocities;
      double velocity_weight;
      bool weight_distances_by_confidences;
      bool weight_distances_by_velocities;
    };

    class OrientationSkeletonTracker
    {
    public:
      OrientationSkeletonTracker();
      ~OrientationSkeletonTracker();

      void configure();
      void start();

    private:
      void stop();
      void setupRosTopics();
      void checkFrameIdConsistency(hiros_skeleton_msgs::OrientationSkeletonGroupConstPtr t_skeleton_group_msg);

      void detectorCallback(hiros_skeleton_msgs::OrientationSkeletonGroupConstPtr t_skeleton_group_msg);
      void trackOldestFrame();

      ros::Time getPreviousSourceTime() const;
      void addNewSkeletonGroupToBuffer(hiros_skeleton_msgs::OrientationSkeletonGroupConstPtr t_skeleton_group_msg);
      void eraseOldSkeletonGroupFromBuffer();
      void mergeTracks(bool& ready_to_be_published);
      void publishTracks(const hiros::skeletons::types::OrientationSkeletonGroup& t_tracks) const;

      void fillDetections();
      void createCostMatrix();
      void solveMunkres();
      void removeDistantMatches();
      void updateDetectedTracks();
      void addNewTracks();
      void removeUnassociatedTracks();

      double computeDistance(const hiros::skeletons::types::OrientationSkeleton& t_track,
                             hiros::skeletons::types::OrientationSkeleton& t_detection) const;
      void updateDetectedTrack(const unsigned int& t_track_idx, const unsigned int& t_det_idx);
      void addNewTrack(const hiros::skeletons::types::OrientationSkeleton& t_detection);
      bool unassociatedDetection(const unsigned int& t_det_idx) const;
      bool unassociatedTrack(const unsigned int& t_track_idx) const;

      void computeAvgTracks();
      double computeAvgSrcTime() const;
      hiros::skeletons::types::OrientationSkeleton
      computeAvgTrack(const std::vector<hiros::skeletons::types::OrientationSkeleton>& t_tracks) const;

      ros::NodeHandle m_nh;
      std::string m_node_namespace;

      OrientationSkeletonTrackerParameters m_params;

      std::string m_frame_id;
      unsigned long m_n_detectors;
      std::vector<ros::Subscriber> m_in_skeleton_group_subs;
      ros::Publisher m_out_msg_pub;

      // vector<pair<src_frame, frame_id>>
      std::vector<std::pair<std::string, std::string>> m_received_frames;

      // map<src_time, orientation_skeleton_group>
      std::map<ros::Time, skeletons::types::OrientationSkeletonGroup> m_skeleton_groups_buffer;

      Munkres m_munkres;
      cv::Mat_<double> m_cost_matrix;
      cv::Mat_<int> m_munkres_matrix;
      int m_last_track_id;

      std::map<int, ros::Time> m_track_id_to_time_stamp_map;

      // vector<pair<src_time, src_frame>>
      std::vector<std::pair<ros::Time, std::string>> m_frames_to_merge;
      // map<track_id, vector<skeleton_tracks>>
      std::map<int, std::vector<skeletons::types::OrientationSkeleton>> m_tracks_to_merge;

      skeletons::types::OrientationSkeletonGroup m_detections{};
      skeletons::types::OrientationSkeletonGroup m_tracks{};
      skeletons::types::OrientationSkeletonGroup m_avg_tracks{};

      bool m_configured;
    };
  } // namespace track
} // namespace hiros

#endif
