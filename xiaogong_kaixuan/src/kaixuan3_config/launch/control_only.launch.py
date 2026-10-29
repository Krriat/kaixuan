"""Minimal ros2_control simulation launch (no MoveIt).

Use this to verify:
    - /joint_states
    - /right_arm_controller/follow_joint_trajectory
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kaixuan1", package_name="kaixuan3_config")
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # 1) Static virtual joint TFs (if any)
    for entity in generate_static_virtual_joint_tfs_launch(moveit_config).entities:
        ld.add_action(entity)

    # 2) Robot state publisher
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            respawn=True,
            parameters=[moveit_config.robot_description, {"publish_frequency": 15.0}],
        )
    )

    # 3) ros2_control_node (mock hardware + controllers)
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            output="screen",
        )
    )

    # 4) Spawn controllers
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )
    )

    return ld
