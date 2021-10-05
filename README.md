# Hi-ROS Skeleton Tracker


## Dependencies
* [Hi-ROS Skeleton Messages](https://gitlab.com/hi-ros/skeleton_msgs)


## Launch files
**default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


## Usage
This ROS package takes as input a SkeletonGroup message and assigns to each skeleton a proper ID.

```
roslaunch hiros_skeleton_tracker custom_configuration_example.launch
```

## Parameters

| Parameter                         | Description                                                  |
| --------------------------------- | ------------------------------------------------------------ |
| `node_required`                   | Set if the other ROS nodes on the PC should be killed when the driver is killed |
| `node_name`                       | Node's name                                                  |
| `in_skeleton_group_topics`        | Topics published by all the detectors in the network         |
| `out_msg_topic_name`              | Name of the topic that will be published containing the tracked skeletons |
| `fixed_delay`                     | Fixed delay to apply before tracking a detection (avoid possible source time inconsistencies when using multiple detectors) |
| `min_markers`                     | Minimum number of markers to be detected to begin tracking   |
| `min_links`                       | Minimum number of links to be detected to begin tracking     |
| `min_linear_distance`             | If the linear distance between an unassociated detection and a track is lower than this parameter, then the detection is considered as part of the track |
| `max_linear_distance`             | Maximum acceptable linear distance between track and detection |
| `min_angular_distance`            | If the angular distance between an unassociated detection and a track is lower than this parameter, then the detection is considered as part of the track |
| `max_angular_distance`            | Maximum acceptable angular distance between track and detection |
| `max_delta_t`                     | Maximum acceptable time delta between track and detection    |
| `use_positions`                   | Calculate distances based on the markers/links positions     |
| `use_linear_velocities`           | Calculate distances based on the markers/links linear velocities |
| `use_orientations`                | Calculate distances based on the markers/links orientations  |
| `use_angular_velocities`          | Calculate distances based on the markers/links angular velocities |
| `velocity_weight`                 | Constant weight to apply to the distance calculated on the velocities |
| `weight_distances_by_confidences` | Weight the distance of each pair of markers/links w.r.t. the detection's confidence |
| `weight_distances_by_velocities`  | Weight the distance of each pair of markers/links w.r.t. the inverse of the track's velocity |

