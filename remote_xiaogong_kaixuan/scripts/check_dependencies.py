#!/usr/bin/env python3
import importlib.util
import sys

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    sys.exit("请先 source /opt/ros/humble/setup.bash")

missing = []
for name in ["rviz2", "moveit_ros_visualization", "moveit_kinematics", "moveit_configs_utils",
             "rmw_fastrtps_cpp", "moveit_msgs", "controller_manager_msgs", "launch_ros"]:
    try:
        get_package_share_directory(name)
    except LookupError:
        missing.append(name)
for name in ["rclpy", "yaml"]:
    if importlib.util.find_spec(name) is None:
        missing.append(name)
if missing:
    print("缺少依赖：" + ", ".join(missing))
    sys.exit("请先运行 bash ./install_dependencies.sh")
