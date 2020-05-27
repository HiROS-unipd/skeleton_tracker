# skeleton_tracker


## Dependencies
* [skeleton\_msgs](https://gitlab.com/hi-ros/skeleton_msgs)


## Launch files
**hiros\_skeleton\_tracker\_default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


## Usage
This ROS package takes as input the skeletons detected by [openpose\_wrapper](https://gitlab.com/hi-ros/openpose_wrapper)
(or by [opw\_3d\_projection](https://gitlab.com/hi-ros/opw_3d_projection)) and assigns to each skeleton a proper track id.

```
roslaunch hiros_openpose_wrapper custom_configuration_example.launch
```

## Parameters
* **fixed_delay**: Fixed delay to apply before tracking a detection (avoid possible source time inconsistencies when using multiple detectors)
* **min_keypoints**: Minimum number of keypoints to be detected to begin tracking
* **max_distance**: Maximum acceptable distance between track and detection
* **max_delta_t**: Maximum acceptable time delta between track and detection
* **use_keypoint_positions**: Calculate distances based on the keypoint positions
* **use_keypoint_velocities**: Calculate distances based on the keypoint velocities
* **velocity_weight**: Constant weight to apply to the distance calculated on the velocities
* **weight_distances_by_velocities**: Weight the distance of each pair of keypoints w.r.t. the inverse of the track's velocity
* **weight_distances_by_confidences**: Weight the distance of each pair of keypoints w.r.t. the detection's confidence
