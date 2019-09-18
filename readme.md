# hj_object_detect

## Overview

This is a ROS package developed for object detection in camera images.
Using [darknet_ros(YOLO)](https://github.com/leggedrobotics/darknet_ros), for real-time object detection system. And using [jsk_pcl](https://github.com/jsk-ros-pkg/jsk_recognition) for estimation coordinates of detected objects, just one class. NOT SUPPORT MULTI CLASSES.
In the following ROS package detecting "plier", "hammer", "screw_driver". for this, I've ref. the [open-manipulator-object-tracking-apple](https://github.com/AuTURBO/open-manipulator-object-tracking-apple).

The packages have been tested under ROS Kinetic and Ubuntu 16.04, 
OpenCV 3.4.2,
NVIDIA-SMI 418.87.00    Driver Version: 418.87.00    CUDA Version: 10.1     


**Author: [Hyeonjun Park](https://www.linkedin.com/in/hyeonjun-park-41bb59125), koreaphj91@gmail.com**

**Affiliation: [Human-Robot Interaction LAB](https://khu-hri.weebly.com), Kyung Hee Unviersity, South Korea**

![hj_object dectect: Detection image](https://user-images.githubusercontent.com/4105524/63675994-008b8700-c825-11e9-84fb-1be015bc3be6.png)


## Installation
- Before do this, please backup important files.

### Dependencies

This software is built on the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Installation)).

One line install: https://cafe.naver.com/openrt/14575 
```
for Desktop

wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
``` 

Additionally, YOLO for ROS depends on following software:

- [OpenCV 3.4.x](http://opencv.org/) (computer vision library),
- [OpenCV 3.4.2](https://jsh93.tistory.com/53) (How to install the opencv? in Korean),
```
$ sudo apt-get update && sudo apt-get upgrade

-- installation the env.
$ sudo apt-get -y purge  libopencv* python-opencv
$ sudo apt-get -y install build-essential cmake vim
$ sudo apt-get -y install pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev
$ sudo apt-get -y install libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev libv4l-dev
$ sudo apt-get -y install v4l-utils libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libqt4-dev
$ sudo apt-get -y install libgtk2.0-dev libgtk-3-dev mesa-utils libgl1-mesa-dri libqt4-opengl-dev
$ sudo apt-get -y install libatlas-base-dev gfortran libeigen3-dev python3-dev python3-numpy python-dev python-numpy libatlas-base-dev gfortran

-- opencv 3.4.2 & opencv_contrib-3.4.2 download
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.2.zip
$ unzip opencv.zip
$ wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.2.zip
$ unzip opencv_contrib.zip

-- opencv build
$ cd ~/opencv-3.4.2/
$ mkdir build
$ cd build
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_TBB=OFF \
-D WITH_IPP=OFF \
-D WITH_1394=OFF \
-D BUILD_WITH_DEBUG_INFO=OFF \
-D BUILD_DOCS=OFF \
-D INSTALL_C_EXAMPLES=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D BUILD_EXAMPLES=OFF \
-D BUILD_TESTS=OFF \
-D BUILD_PERF_TESTS=OFF \
-D WITH_QT=OFF \
-D WITH_GTK=ON \
-D WITH_OPENGL=ON \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.2/modules \
-D WITH_V4L=ON  \
-D WITH_FFMPEG=ON \
-D WITH_XINE=ON \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
../

-- opencv make compile
$ time make

-- opencv install
$ sudo make install
```
- [CUDA 10.0 with cuDNN 10.x](https://greedywyatt.tistory.com/106), (How to install the cuda? in Korean),
   - notice: Do it after install the nvidia driver. 
             And the cuda version is follow on your GPU. check up the [GPUs supported](https://en.wikipedia.org/wiki/CUDA)
- Download the CUDA in the NVIDIA webpage ([link](https://developer.nvidia.com/cuda-10.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=deblocal))
```
$ sudo dpkg -i cuda-repo-ubuntu1604-10-0-local-10.0.130-410.48_1.0-1_amd64.deb
$ sudo apt-key add /var/cuda-repo-<version>/7fa2af80.pub
$ sudo apt-get update
$ sudo apt-get install cuda

$ gedit ~/.bashrc
   export PATH=/usr/local/cuda-10.0/bin${PATH:+:${PATH}}$ 
   export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

Or, if you installed the cuda 10.1,
$ gedit ~/.bashrc
   export PATH=/usr/local/cuda-10.1/bin${PATH:+:${PATH}}$ 
   export LD_LIBRARY_PATH=/usr/local/cuda-10.1/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

$ reboot

- check up the installed
$ cat /proc/driver/nvidia/version
   NVRM version: NVIDIA UNIX x86_64 Kernel Module  418.87.00  Thu Aug  8 15:35:46 CDT 2019
   GCC version:  gcc version 5.4.0 20160609 (Ubuntu 5.4.0-6ubuntu1~16.04.11)
$ nvcc -V
   nvcc: NVIDIA (R) Cuda compiler driver
   Copyright (c) 2005-2018 NVIDIA Corporation
   Built on Sat_Aug_25_21:08:01_CDT_2018
   Cuda compilation tools, release 10.0, V10.0.130

- After download the cuDNN
$ sudo dpkg -i libcudnn7_7.6.2.24-1+cuda10.0_amd64.deb
$ sudo dpkg -i libcudnn7-dev_7.6.2.24-1+cuda10.0_amd64.deb
$ sudo dpkg -i libcudnn7-doc_7.6.2.24-1+cuda10.0_amd64.deb
```

- [darknet YOLO](https://juni-94.tistory.com/9), (How to install the YOLO? in Korean),

- [darknet_ros](https://qiita.com/nnn112358/items/d696681d5b0577d633b6) (How to install the darknet_ros? in Japanese)
```
 $ cd ~/catkin_ws/src 
 $ git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
 $ cd ..
 $ catkin_make darknet_ros
 $ cd ~/catkin_ws/devel
 $ source setup.bash 
```

- [Intel® RealSense™ Depth Camera D435 library](http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_applications/#ros-applications)

## How to start?
Copy and paste
```
$ roscd hj_object_detect/
$ cd copy_paste_files
$ cp -r hj_p.yaml ~/catkin_ws/src/darknet_ros/darknet_ros/config
$ cp -r p.cfg ~/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/cfg
$ cp -r YoloObjectDetector.cpp ~/catkin_ws/src/darknet_ros/darknet_ros/src
```

Real realsense camera
```
$ roslaunch hj_object_detect hj_object_detect_rviz.launch 
$ roslaunch hj_object_detect hj_jsk_test.launch
```

Gazebo simulation
```
$ roslaunch hj_object_detect hj_object_detect_rviz.launch sim:=true
$ roslaunch hj_object_detect hj_jsk_test.launch sim:=true
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

## Video
Click image to link to YouTube video.  
[![plier_check](https://user-images.githubusercontent.com/4105524/63911188-631e9600-ca64-11e9-9825-16f701b9bb00.png)](https://youtu.be/3cFitqKaLN0)   


