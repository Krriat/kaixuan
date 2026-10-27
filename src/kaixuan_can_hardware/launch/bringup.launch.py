from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("kaixuan_can_hardware"), "config", "ros2_controllers.yaml"]
    )

    robot_xacro = PathJoinSubstitution(
        [FindPackageShare("kaixuan_can_hardware"), "urdf", "kaixuan1_ros2_control_real.urdf.xacro"]
    )

    cycle_time_ns = LaunchConfiguration("cycle_time_ns")
    shift_time_ns = LaunchConfiguration("shift_time_ns")
    priority = LaunchConfiguration("priority")
    master0_affinity = LaunchConfiguration("master0_affinity")
    master1_affinity = LaunchConfiguration("master1_affinity")
    o6_command_timeout_ms = LaunchConfiguration("o6_command_timeout_ms")
    o6_max_step = LaunchConfiguration("o6_max_step")
    o6_imu_only = LaunchConfiguration("o6_imu_only")

    # Optional per-joint compensation in degrees
    comp0 = LaunchConfiguration("comp0")
    comp1 = LaunchConfiguration("comp1")
    comp2 = LaunchConfiguration("comp2")
    comp3 = LaunchConfiguration("comp3")
    comp4 = LaunchConfiguration("comp4")
    comp5 = LaunchConfiguration("comp5")
    comp6 = LaunchConfiguration("comp6")
    left_comp0 = LaunchConfiguration("left_comp0")
    left_comp1 = LaunchConfiguration("left_comp1")
    left_comp2 = LaunchConfiguration("left_comp2")
    left_comp3 = LaunchConfiguration("left_comp3")
    left_comp4 = LaunchConfiguration("left_comp4")
    left_comp5 = LaunchConfiguration("left_comp5")
    left_comp6 = LaunchConfiguration("left_comp6")
    right_gear_ratios = [LaunchConfiguration(f"right_gear_ratio{i}") for i in range(1, 8)]
    left_gear_ratios = [LaunchConfiguration(f"left_gear_ratio{i}") for i in range(1, 8)]

    xacro_command = [
        "xacro",
        " ",
        robot_xacro,
        " ", "cycle_time_ns:=", cycle_time_ns,
        " ", "shift_time_ns:=", shift_time_ns,
        " ", "priority:=", priority,
        " ", "master0_affinity:=", master0_affinity,
        " ", "master1_affinity:=", master1_affinity,
        " ", "o6_command_timeout_ms:=", o6_command_timeout_ms,
        " ", "o6_max_step:=", o6_max_step,
        " ", "o6_imu_only:=", o6_imu_only,
        " ", "comp0:=", comp0,
        " ", "comp1:=", comp1,
        " ", "comp2:=", comp2,
        " ", "comp3:=", comp3,
        " ", "comp4:=", comp4,
        " ", "comp5:=", comp5,
        " ", "comp6:=", comp6,
        " ", "left_comp0:=", left_comp0,
        " ", "left_comp1:=", left_comp1,
        " ", "left_comp2:=", left_comp2,
        " ", "left_comp3:=", left_comp3,
        " ", "left_comp4:=", left_comp4,
        " ", "left_comp5:=", left_comp5,
        " ", "left_comp6:=", left_comp6,
    ]
    for i, ratio in enumerate(right_gear_ratios, start=1):
        xacro_command.extend([" ", f"right_gear_ratio{i}:=", ratio])
    for i, ratio in enumerate(left_gear_ratios, start=1):
        xacro_command.extend([" ", f"left_gear_ratio{i}:=", ratio])

    robot_description = {
        "robot_description": Command(xacro_command)
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    home_server = Node(
        package="kaixuan_can_hardware",
        executable="kaixuan_home_server",
        output="screen",
        parameters=[
            {"action_name": "/right_arm_controller/follow_joint_trajectory"},
            {
                "joints": [
                    "right_joint1",
                    "right_joint2",
                    "right_joint3",
                    "right_joint4",
                    "right_joint5",
                    "right_joint6",
                    "right_joint7",
                ]
            },
            {"home_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            {"move_time": 5.0},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("cycle_time_ns", default_value="1000000"),
            DeclareLaunchArgument("shift_time_ns", default_value="0"),
            DeclareLaunchArgument("priority", default_value="90"),
            DeclareLaunchArgument("master0_affinity", default_value="6"),
            DeclareLaunchArgument("master1_affinity", default_value="7"),
            DeclareLaunchArgument("o6_command_timeout_ms", default_value="0"),
            DeclareLaunchArgument("o6_max_step", default_value="255"),
            DeclareLaunchArgument("o6_imu_only", default_value="false"),
            DeclareLaunchArgument("comp0", default_value="0"),
            DeclareLaunchArgument("comp1", default_value="0"),
            DeclareLaunchArgument("comp2", default_value="0"),
            DeclareLaunchArgument("comp3", default_value="0"),
            DeclareLaunchArgument("comp4", default_value="0"),
            DeclareLaunchArgument("comp5", default_value="0"),
            DeclareLaunchArgument("comp6", default_value="0"),
            DeclareLaunchArgument("left_comp0", default_value="0"),
            DeclareLaunchArgument("left_comp1", default_value="0"),
            DeclareLaunchArgument("left_comp2", default_value="0"),
            DeclareLaunchArgument("left_comp3", default_value="0"),
            DeclareLaunchArgument("left_comp4", default_value="0"),
            DeclareLaunchArgument("left_comp5", default_value="0"),
            DeclareLaunchArgument("left_comp6", default_value="0"),
            *[
                DeclareLaunchArgument(f"right_gear_ratio{i}", default_value=ratio)
                for i, ratio in enumerate(
                    ("101.0", "101.0", "81.0", "81.0", "101.0", "101.0", "101.0"),
                    start=1,
                )
            ],
            *[
                DeclareLaunchArgument(f"left_gear_ratio{i}", default_value=ratio)
                for i, ratio in enumerate(
                    ("101.0", "101.0", "81.0", "81.0", "101.0", "101.0", "101.0"),
                    start=1,
                )
            ],
            control_node,
            joint_state_broadcaster_spawner,
            right_arm_controller_spawner,
            left_arm_controller_spawner,
            home_server,
        ]
    )
