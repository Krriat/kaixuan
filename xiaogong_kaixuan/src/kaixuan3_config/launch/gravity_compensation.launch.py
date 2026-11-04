"""Load gravity compensation inactive, with the left arm selected by default."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = get_package_share_directory("kaixuan3_config")
    hardware = get_package_share_directory("kaixuan_can_hardware")
    return LaunchDescription([
        DeclareLaunchArgument("joints", default_value=",".join(f"left_joint{i}" for i in range(1, 8))),
        DeclareLaunchArgument("use_mock_hardware", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("torque_config_file", default_value=os.path.join(hardware, "config", "gravity_limits.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(config, "launch", "torque_control.launch.py")),
            launch_arguments={
                "control_kind": "gravity",
                "joints": LaunchConfiguration("joints"),
                "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
                "torque_config_file": LaunchConfiguration("torque_config_file"),
            }.items()),
    ])
