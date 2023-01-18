# Lidar2Depth
Internal repository for projecting a lidar point cloud to a depth map.

## Installation

```
cd ~/catkin_ws/src
git clone https://github.com/zaalsabb/lidar2depth.git
cd ..
catkin_make
```

## ROS Requirements
```
sudo apt-get install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-pcl-conversions ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-message-filters ros-$ROS_DISTRO-image-transport* python-catkin-tools
```

## Running

```
roslaunch lidar2depth bag2depth.launch save_directory:=DATA_DIR max_cloud_size:=1000000 future_slider:=0.2 depth_rescale:=0.5
```

Your data should be placed in `DATA_DIR`, which is set by default to `/home/user/datasets/processed`.
