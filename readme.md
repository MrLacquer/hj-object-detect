# hj_object_detect

## Overview

This is a ROS package developed for object detection in camera images.
Using [darknet_ros(YOLO)](https://github.com/leggedrobotics/darknet_ros), for real-time object detection system. And using [jsk_pcl](https://github.com/jsk-ros-pkg/jsk_recognition) for estimation coordinates of detected objects, just one class. NOT SUPPORT MULTI CLASSES.
In the following ROS package detecting "plier", "hammer", "screw_driver". for this, I've ref. the [open-manipulator-object-tracking-apple](https://github.com/AuTURBO/open-manipulator-object-tracking-apple).

The packages have been tested under ROS Kinetic and Ubuntu 16.04. 

**Author: [Hyeonjun Park](), koreaphj91@gmail.com**

**Affiliation: [Human-Robot Interaction LAB](https://khu-hri.weebly.com), Kyung Hee Unviersity, South Korea**

![hj_object dectect: Detection image](hj_object_detect/docdeteced_object.png)

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionally, YOLO for ROS depends on following software:

- [OpenCV](http://opencv.org/) (computer vision library),
- [boost](http://www.boost.org/) (c++ library),

## How to start?
```
$ roslaunch hj_object_detect hj_object_detect.launch 
$ roslaunch hj_object_detect hj_jsk_test.launch
```

## Launch tree

### hj_object_detect.launch 
- bringup_d435.launch
   - rs_rgbd.launch
- hj_darknet.launch
   - config: $(find darknet_ros)/yolo_network_config/weights
   - weight: $(find darknet_ros)/yolo_network_config/cfg
   - parameter: $(find hj_object_detect)/config/hj_darknet_config.yaml
   - parameter: $(find darknet_ros)/config/hj_p.yaml

### hj_jsk_test.launch
- point cloud nodelet
- jsk_pcl_utils/LabelToClusterPointIndices nodelet
- jsk_pcl/ClusterPointIndicesDecomposer