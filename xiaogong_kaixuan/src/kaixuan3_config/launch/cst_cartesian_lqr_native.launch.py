"""Optional experimental 1 kHz LQI bridge. Launch alone does not enable drives."""
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
        DeclareLaunchArgument("lqr_config_file", default_value=os.path.join(
            config, "config", "cst_cartesian_lqr_native.yaml")),
        DeclareLaunchArgument("shadow_only", default_value="false", choices=["false", "true"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(config, "launch", "torque_control.launch.py")),
            launch_arguments={
                "control_kind": "torque_lqr",
                "joints": ",".join(f"left_joint{i}" for i in range(1, 8)),
                "use_mock_hardware": "false",
                "stop_cst_group_on_fault": "true",
                "command_timeout": "0.1",
                "cst_disabled_control_word": "0",
                "torque_config_file": os.path.join(hardware, "config", "cartesian_native_torque_limits.yaml"),
                "lqr_config_file": LaunchConfiguration("lqr_config_file"),
                "lqr_shadow_only": LaunchConfiguration("shadow_only"),
            }.items()),
    ])
