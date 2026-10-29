import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def _load_yaml(package_name: str, rel_path: str):
    path = os.path.join(get_package_share_directory(package_name), rel_path)
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("use_real_hardware", default_value=False))
    ld.add_action(DeclareLaunchArgument("can0_iface", default_value="can0"))
    ld.add_action(DeclareLaunchArgument("can1_iface", default_value="can1"))
    for i in range(7):
        ld.add_action(DeclareLaunchArgument(f"comp{i}", default_value="0"))

    use_real = LaunchConfiguration("use_real_hardware")

    sim_urdf_xacro = os.path.join(
        get_package_share_directory("kaixuan3_config"), "config", "kaixuan1.urdf.xacro"
    )
    real_urdf_xacro = os.path.join(
        get_package_share_directory("kaixuan_can_hardware"),
        "urdf",
        "kaixuan1_ros2_control_real.urdf.xacro",
    )

    sim_moveit_config = (
        MoveItConfigsBuilder("kaixuan1", package_name="kaixuan3_config")
        .robot_description(file_path=sim_urdf_xacro)
        .to_moveit_configs()
    )
    real_moveit_config = (
        MoveItConfigsBuilder("kaixuan1", package_name="kaixuan3_config")
        .robot_description(
            file_path=real_urdf_xacro,
            mappings={
                "can0_iface": LaunchConfiguration("can0_iface"),
                "can1_iface": LaunchConfiguration("can1_iface"),
                **{f"comp{i}": LaunchConfiguration(f"comp{i}") for i in range(7)},
            },
        )
        .to_moveit_configs()
    )

    servo_yaml = _load_yaml("kaixuan3_config", os.path.join("config", "gp110_simulated_config.yaml"))
    # move_group 已经在 moveit_bringup 里运行，这里让 Servo 不做 primary planning scene
    servo_yaml["is_primary_planning_scene_monitor"] = False
    servo_params = {"moveit_servo": servo_yaml}

    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="servo_node",
            output="screen",
            parameters=[
                servo_params,
                sim_moveit_config.robot_description,
                sim_moveit_config.robot_description_semantic,
                sim_moveit_config.robot_description_kinematics,
            ],
            condition=UnlessCondition(use_real),
        )
    )

    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="servo_node",
            output="screen",
            parameters=[
                servo_params,
                real_moveit_config.robot_description,
                real_moveit_config.robot_description_semantic,
                real_moveit_config.robot_description_kinematics,
            ],
            condition=IfCondition(use_real),
        )
    )

    return ld
