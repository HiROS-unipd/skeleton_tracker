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
