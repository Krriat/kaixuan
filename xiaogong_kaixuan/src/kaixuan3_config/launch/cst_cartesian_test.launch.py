"""Seven-axis CST bridge; launch does not arm torque output.

Electrical disable is not proof of mechanical holding: loaded-arm tests found
passive joint motion after both CW6 and CW0. See docs/CST_LQR_VIBRATION.md.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config = get_package_share_directory("kaixuan3_config")
    hardware = get_package_share_directory("kaixuan_can_hardware")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(config, "launch", "torque_control.launch.py")),
            launch_arguments={
                "control_kind": "torque",
                "joints": ",".join(f"left_joint{i}" for i in range(1, 8)),
                "use_mock_hardware": "false",
                "stop_cst_group_on_fault": "true",
                "command_timeout": "0.1",
                "torque_config_file": os.path.join(hardware, "config", "cartesian_torque_limits.yaml"),
            }.items()),
    ])
