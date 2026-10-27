"""RViz-only launch for remote MoveIt/ros2_control setup.

Use this on a PC when robot side already runs move_group + ros2_control.
This launch starts only rviz2 and injects the robot/moveit parameters needed by
MotionPlanning display.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("use_real_hardware", default_value=True))

    ld.add_action(DeclareLaunchArgument("cycle_time_ns", default_value="1000000"))
    ld.add_action(DeclareLaunchArgument("shift_time_ns", default_value="0"))
    ld.add_action(DeclareLaunchArgument("priority", default_value="90"))
    ld.add_action(DeclareLaunchArgument("master0_affinity", default_value="6"))
    ld.add_action(DeclareLaunchArgument("master1_affinity", default_value="7"))

    comp_defaults = [
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
    ]
    left_comp_defaults = [
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
    ]
    for i, default in enumerate(comp_defaults):
        ld.add_action(DeclareLaunchArgument(f"comp{i}", default_value=default))
        ld.add_action(DeclareLaunchArgument(f"left_comp{i}", default_value=left_comp_defaults[i]))

    use_real = LaunchConfiguration("use_real_hardware")

    sim_urdf_xacro = os.path.join(
        get_package_share_directory("kaixuan3_config"), "config", "kaixuan1.urdf.xacro"
    )
    sim_initial_positions = os.path.join(
        get_package_share_directory("kaixuan3_config"), "config", "initial_positions.yaml"
    )

    try:
        real_urdf_xacro = os.path.join(
            get_package_share_directory("kaixuan_can_hardware"),
            "urdf",
            "kaixuan1_ros2_control_real.urdf.xacro",
        )
    except Exception:
        real_urdf_xacro = sim_urdf_xacro  # kaixuan_can_hardware not installed

    sim_mappings = {
        "initial_positions_file": sim_initial_positions,
    }
    real_mappings = {
        "cycle_time_ns": LaunchConfiguration("cycle_time_ns"),
        "shift_time_ns": LaunchConfiguration("shift_time_ns"),
        "priority": LaunchConfiguration("priority"),
        "master0_affinity": LaunchConfiguration("master0_affinity"),
        "master1_affinity": LaunchConfiguration("master1_affinity"),
        **{f"comp{i}": LaunchConfiguration(f"comp{i}") for i in range(7)},
        **{f"left_comp{i}": LaunchConfiguration(f"left_comp{i}") for i in range(7)},
    }

    sim_moveit_config = (
        MoveItConfigsBuilder("kaixuan1", package_name="kaixuan3_config")
        .robot_description(file_path=sim_urdf_xacro, mappings=sim_mappings)
        .to_moveit_configs()
    )
    try:
        real_moveit_config = (
            MoveItConfigsBuilder("kaixuan1", package_name="kaixuan3_config")
            .robot_description(file_path=real_urdf_xacro, mappings=real_mappings)
            .to_moveit_configs()
        )
    except Exception:
        real_moveit_config = sim_moveit_config  # fallback when hardware pkg unavailable

    rviz_config = os.path.join(
        get_package_share_directory("kaixuan3_config"), "config", "moveit.rviz"
    )

    rviz_sim = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            sim_moveit_config.robot_description,
            sim_moveit_config.robot_description_semantic,
            sim_moveit_config.planning_pipelines,
            sim_moveit_config.robot_description_kinematics,
            sim_moveit_config.joint_limits,
        ],
        condition=UnlessCondition(use_real),
    )

    rviz_real = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            real_moveit_config.robot_description,
            real_moveit_config.robot_description_semantic,
            real_moveit_config.planning_pipelines,
            real_moveit_config.robot_description_kinematics,
            real_moveit_config.joint_limits,
        ],
        condition=IfCondition(use_real),
    )

    # Launch one RViz instance depending on use_real_hardware.
    # We intentionally avoid starting move_group / ros2_control / robot_state_publisher here.
    # RViz connects to remote robot-side nodes through ROS 2 network.
    ld.add_action(rviz_sim)
    ld.add_action(rviz_real)

    return ld
