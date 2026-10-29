#!/usr/bin/env bash
set -euo pipefail
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "请先安装 ROS 2 Humble。此脚本只补充 RViz / MoveIt 客户端依赖。"
  exit 1
fi
sudo apt-get update
sudo apt-get install -y \
  ros-humble-rviz2 \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-configs-utils \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-controller-manager-msgs \
  ros-humble-moveit-msgs \
  ros-humble-rclpy \
  ros-humble-ros2launch \
  python3-yaml
