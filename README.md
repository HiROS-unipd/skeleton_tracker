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
| `camera_frequency`                | Frame rate of the cameras being used (optional)              |
| `fixed_delay`                     | Fixed delay to apply before tracking a detection (avoid possible source time inconsistencies when using multiple detectors) |
| `min_joints`                      | Minimum number of markers and/or orientations to be detected to begin tracking |
| `min_markers_distance`            | If the distance between an unassociated detection and a track is lower than this parameter, then the detection is considered as part of the track |
| `max_markers_distance`            | Maximum acceptable distance between track and detection      |
| `min_orientations_distance`       | If the distance between an unassociated detection and a track is lower than this parameter, then the detection is considered as part of the track |
| `max_orientations_distance`       | Maximum acceptable distance between track and detection      |
| `max_delta_t`                     | Maximum acceptable time delta between track and detection    |
| `use_markers`                     | Calculate distances based on the markers' positions          |
| `use_markers_velocities`          | Calculate distances based on the markers' velocities         |
| `use_orientations`                | Calculate distances based on the orientations                |
| `use_orientations_velocities`     | Calculate distances based on the angular velocities          |
| `velocity_weight`                 | Constant weight to apply to the distance calculated on the velocities |
| `weight_distances_by_confidences` | Weight the distance of each pair of markers/orientations w.r.t. the detection's confidence |
| `weight_distances_by_velocities`  | Weight the distance of each pair of markers/orientations w.r.t. the inverse of the track's velocity |

