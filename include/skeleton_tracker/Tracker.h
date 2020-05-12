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

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace track {

    struct TrackerParameters
    {
      std::string node_name;

      std::string in_skeleton_group_topic;
      std::string out_msg_topic_name;

      int min_keypoints;
      double max_distance;
      ros::Duration max_delta_t;
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

      std::string extractImageQualityFromTopicName(const std::string& t_topic_name) const;

      void trackingCallback(const skeleton_msgs::SkeletonGroupConstPtr t_skeleton_group_msg);
      void track();

      void fillDetections();
      void createDistanceMatrix();
      void createCostMatrix();
      void solveMunkres();
      void removeDistantMatches();
      void updateDetectedTracks();
      void addNewTracks();
      void removeUnassociatedTracks();

      double computeDistance(const hiros::skeletons::types::Skeleton& t_track,
                             const hiros::skeletons::types::Skeleton& t_detection) const;
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

      ros::Subscriber m_in_skeleton_group_sub;
      ros::Publisher m_out_msg_pub;

      ros::Time m_skeleton_group_src_time;
      skeletons::types::SkeletonGroup m_skeleton_group;

      cv::Mat_<double> m_distance_matrix;
      cv::Mat_<double> m_cost_matrix;
      cv::Mat_<int> m_munkres_matrix;
      int m_last_track_id;

      std::map<int, ros::Time> m_track_id_to_time_stamp_map;
      std::shared_ptr<skeletons::types::SkeletonGroup> m_tracks;
      std::shared_ptr<skeletons::types::SkeletonGroup> m_detections;

      bool m_configured;
    };
  } // namespace track
} // namespace hiros

#endif
