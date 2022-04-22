#!/bin/bash

base_debs="
  software-properties-common
  curl
  chrony
  openssh-server
  build-essential
"

# system python 2 debs
sys_py2_debs="
  python-pip
  python-sklearn
  python-lxml
  python-numpy
"

sys_py3_debs="
  python3-pip
  python3-sklearn
  python3-lxml
"

ros_debs="
  ros-$ROS_DISTRO-roslint
  ros-$ROS_DISTRO-ros-numpy
  ros-$ROS_DISTRO-ros-control
  ros-$ROS_DISTRO-ros-controllers
  ros-$ROS_DISTRO-pcl-ros
  ros-$ROS_DISTRO-industrial-core
  ros-$ROS_DISTRO-rosbridge-server
  ros-$ROS_DISTRO-map-server
  ros-$ROS_DISTRO-octomap-ros
  ros-$ROS_DISTRO-robot-localization
  ros-$ROS_DISTRO-gazebo-msgs
  ros-$ROS_DISTRO-tf2-sensor-msgs
  ros-$ROS_DISTRO-ddynamic-reconfigure
  ros-$ROS_DISTRO-joint-state-publisher-gui
  ros-$ROS_DISTRO-rgbd-launch
  ros-$ROS_DISTRO-laser-geometry
  ros-$ROS_DISTRO-laser-assembler
  ros-$ROS_DISTRO-rviz-visual-tools
  ros-$ROS_DISTRO-sick-scan
  ros-$ROS_DISTRO-sick-safetyscanners
"

ros_noetic_debs="
"

developer_tools="
  vim
  htop
  tmux
  meld
  git-gui
"

