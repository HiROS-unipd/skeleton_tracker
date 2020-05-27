#ifndef hiros_skeleton_tracker_Tracker_h
#define hiros_skeleton_tracker_Tracker_h

// ROS dependencies
#include <ros/ros.h>

// OpenCV dependencies
#include "opencv2/opencv.hpp"

// Custom Ros Message dependencies
#include "skeleton_msgs/SkeletonGroup.h"

// Custom External Packages dependencies
#include "skeletons/types.h"

// Custom Internal dependencies
#include "skeleton_tracker/Munkres.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace track {

    struct TrackerParameters
    {
      std::string node_name;

      std::vector<std::string> in_skeleton_group_topics;
      std::string out_msg_topic_name;

      int min_keypoints;
      double max_distance;
      ros::Duration max_delta_t;
      bool use_keypoint_positions;
      bool use_keypoint_velocities;
      double velocity_weight;
      bool weight_distances_by_confidences;
      bool weight_distances_by_velocities;
    };

    class Tracker
    {
    public:
      Tracker();
      ~Tracker();

      void configure();
      void start();

    private:
      void stop();
      void setupRosTopics();

      std::string extractImageQualityFromTopicNames(const std::vector<std::string>& t_topic_names) const;

      void detectorCallback(const skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg);
      void track();

      void fillDetections();
      void createCostMatrix();
      void solveMunkres();
      void removeDistantMatches();
      void updateDetectedTracks();
      void addNewTracks();
      void removeUnassociatedTracks();

      double computeDistance(const hiros::skeletons::types::Skeleton& t_track,
                             hiros::skeletons::types::Skeleton& t_detection);
      void computeVelAndAcc(const hiros::skeletons::types::Skeleton& t_track,
                            hiros::skeletons::types::Skeleton& t_detection);
      void computeVelocities(const hiros::skeletons::types::Skeleton& t_track,
                             hiros::skeletons::types::Skeleton& t_detection);
      hiros::skeletons::types::Velocity computeVelocity(const hiros::skeletons::types::Point& t_prev,
                                                        const hiros::skeletons::types::Point& t_curr,
                                                        const double& t_dt);
      hiros::skeletons::types::Acceleration computeAcceleration(const hiros::skeletons::types::Point& t_prev,
                                                                const hiros::skeletons::types::Point& t_curr,
                                                                const double& t_dt);
      void updateDetectedTrack(const unsigned int& t_track_idx, const unsigned int& t_det_idx);
      void addNewTrack(const hiros::skeletons::types::Skeleton& t_detection);
      bool unassociatedDetection(const unsigned int& t_det_idx) const;
      bool unassociatedTrack(const unsigned int& t_track_idx) const;

      bool match(const unsigned int& t_track_idx, const unsigned int& t_det_idx) const;
      bool colHasMatch(const unsigned int& t_col) const;
      int findMatchInCol(const unsigned int& t_col) const;
      bool rowHasMatch(const unsigned int& t_row) const;
      int findMatchInRow(const unsigned int& t_row) const;

      ros::NodeHandle m_nh;
      std::string m_node_namespace;

      TrackerParameters m_params;

      std::vector<ros::Subscriber> m_in_skeleton_group_subs;
      ros::Publisher m_out_msg_pub;

      ros::Time m_prev_skeleton_group_src_time;
      ros::Time m_skeleton_group_src_time;
      skeletons::types::SkeletonGroup m_skeleton_group;

      Munkres m_munkres;
      cv::Mat_<double> m_cost_matrix;
      cv::Mat_<int> m_munkres_matrix;
      int m_last_track_id;

      std::map<int, ros::Time> m_track_id_to_time_stamp_map;
      skeletons::types::SkeletonGroup m_tracks;
      skeletons::types::SkeletonGroup m_detections;

      bool m_configured;
    };
  } // namespace track
} // namespace hiros

#endif
