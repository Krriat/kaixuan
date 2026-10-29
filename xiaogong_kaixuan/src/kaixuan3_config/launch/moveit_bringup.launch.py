"""kaixuan3_config bringup for MoveIt + ros2_control.

This launch provides a single switch to run either:
    - Simulation (mock_components/GenericSystem)
    - Real hardware (kaixuan_can_hardware/KaixuanCanSystem via direct IgH EtherCAT)

Switch:
    use_real_hardware:=false  -> simulation
    use_real_hardware:=true   -> real robot

Both modes expose per-arm FollowJointTrajectory actions.
"""

import os
import json
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.logging import launch_config

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def start_flight_recorder(context):
    if LaunchConfiguration("record_diagnostics").perform(context).lower() not in {"true", "1", "yes", "on"}:
        return []
    real = LaunchConfiguration("use_real_hardware").perform(context).lower() in {"true", "1", "yes", "on"}
    directory = Path(launch_config.log_dir) / "flight_recorder"
    names = ["use_real_hardware", "active_arm", "cycle_time_ns", "shift_time_ns", "priority",
             "master0_affinity", "master1_affinity", "o6_command_timeout_ms", "o6_max_step",
             "o6_imu_only", "bus_diagnostics"]
    names += [f"{prefix}{i}" for prefix in ("comp", "left_comp") for i in range(7)]
    names += [f"{arm}_gear_ratio{i}" for arm in ("right", "left") for i in range(1, 8)]
    arguments = ["--output-dir", str(directory), "--period", "0.2",
                 "--active-arm", LaunchConfiguration("active_arm").perform(context),
                 "--launch-arguments", json.dumps({name: LaunchConfiguration(name).perform(context) for name in names})]
    if real:
        arguments += ["--real-hardware"]
        if LaunchConfiguration("bus_diagnostics").perform(context).lower() in {"true", "1", "yes", "on"}:
            arguments += ["--expect-bus-diagnostics"]
    config_share = Path(get_package_share_directory("kaixuan3_config"))
    hardware_share = Path(get_package_share_directory("kaixuan_can_hardware"))
    for filename in [
        hardware_share / "config/ros2_controllers.yaml" if real else config_share / "config/ros2_controllers.yaml",
        config_share / "config/joint_limits.yaml",
        config_share / "config/moveit_controllers.yaml",
        hardware_share / "urdf/kaixuan1_ros2_control_real.urdf.xacro",
        hardware_share.parents[1] / "include/kaixuan_can_hardware/bus_probe.hpp",
        config_share.parents[1] / "lib/kaixuan3_config/bus_diagnostics.py",
        config_share.parents[1] / "lib/kaixuan3_config/monitor64.py",
        Path(__file__),
    ]:
        arguments += ["--config", str(filename)]
    return [Node(package="kaixuan3_config", executable="flight_recorder.py",
                 arguments=arguments, output="screen")]


def release_direct_helpers(context):
    use_real = LaunchConfiguration("use_real_hardware").perform(context).lower()
    if use_real not in {"1", "true", "yes", "on"}:
        return []

    active_arm = LaunchConfiguration("active_arm").perform(context).lower()
    active_master_ids = {"both": (0, 1), "right": (0,), "left": (1,)}[active_arm]
    state_dir = Path("/tmp/ethercat_op_csp")
    running = []
    for master_id in active_master_ids:
        pid_path = state_dir / f"master{master_id}.pid"
        try:
            pid = int(pid_path.read_text().strip())
        except (OSError, ValueError):
            continue
        try:
            arguments = [
                item.decode(errors="replace")
                for item in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
                if item
            ]
        except OSError:
            continue
        if not arguments or os.path.basename(arguments[0]) != "ethercat_direct_tool":
            continue
        try:
            helper_master = arguments[arguments.index("--master_id") + 1]
        except (ValueError, IndexError):
            continue
        if helper_master != str(master_id):
            continue
        running.append((master_id, pid))

    if not running:
        return []

    try:
        for master_id, _pid in running:
            (state_dir / f"master{master_id}.stop").touch()
    except PermissionError as exc:
        raise RuntimeError(
            "Cannot release root-owned EtherCAT preparation helpers. Run "
            "'sudo python3 src/normal/prepare_ethercat_op_csp.py --stop' first."
        ) from exc

    deadline = time.monotonic() + 8.0
    remaining = list(running)
    while remaining and time.monotonic() < deadline:
        next_remaining = []
        for master_id, pid in remaining:
            if Path(f"/proc/{pid}").exists():
                next_remaining.append((master_id, pid))
        remaining = next_remaining
        if remaining:
            time.sleep(0.1)

    if remaining:
        details = ", ".join(f"master{master_id}:pid{pid}" for master_id, pid in remaining)
        raise RuntimeError(
            f"EtherCAT preparation helpers did not release the masters: {details}. "
            "Run prepare_ethercat_op_csp.py --stop with sudo."
        )
    return []


def generate_launch_description():
    ld = LaunchDescription()

    # ===== Switch =====
    ld.add_action(DeclareBooleanLaunchArg("use_real_hardware", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("record_diagnostics", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("bus_diagnostics", default_value=True))

    # RViz on/off
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(
        DeclareLaunchArgument(
            "active_arm",
            default_value="both",
            choices=["both", "right", "left"],
            description="EtherCAT arm(s) to initialize: both, right (master0), or left (master1)",
        )
    )

    # ===== Real-hardware args (EtherCAT + per-joint compensation, degrees) =====
    ld.add_action(DeclareLaunchArgument("cycle_time_ns", default_value="1000000"))
    ld.add_action(DeclareLaunchArgument("shift_time_ns", default_value="0"))
    ld.add_action(DeclareLaunchArgument("priority", default_value="90"))
    ld.add_action(DeclareLaunchArgument("master0_affinity", default_value="6"))
    ld.add_action(DeclareLaunchArgument("master1_affinity", default_value="7"))
    ld.add_action(DeclareLaunchArgument("o6_command_timeout_ms", default_value="0"))
    ld.add_action(DeclareLaunchArgument("o6_max_step", default_value="255"))
    ld.add_action(DeclareLaunchArgument("o6_imu_only", default_value="false"))
    _comp_defaults = [
        "8.61927750",
        "-115.05901903",
        "-151.79905785",
        "43.58607398",
        "125.67501068",
        "-9.17790101",
        "-0.02567102",
    ]
    # Encoder offsets from the fixed physical pose in config/left_arm_calibration.json.
    # q_deg = raw_output_deg - comp_deg (left direction is +1).
    _left_comp_defaults = [
        "211.82688283",
        "160.58685241",
        "-69.66883257",
        "-98.34962379",
        "-87.44871777",
        "53.70773232",
        "-83.61559409",
    ]
    for i, default in enumerate(_comp_defaults):
        ld.add_action(DeclareLaunchArgument(f"comp{i}", default_value=default))
        ld.add_action(DeclareLaunchArgument(f"left_comp{i}", default_value=_left_comp_defaults[i]))
    _gear_ratio_defaults = ["101.0", "101.0", "81.0", "81.0", "101.0", "101.0", "101.0"]
    for i, default in enumerate(_gear_ratio_defaults, start=1):
        ld.add_action(DeclareLaunchArgument(f"right_gear_ratio{i}", default_value=default))
        ld.add_action(DeclareLaunchArgument(f"left_gear_ratio{i}", default_value=default))

    # ===== Sim args =====
    sim_initial_positions = os.path.join(
        get_package_share_directory("kaixuan3_config"), "config", "initial_positions.yaml"
    )
    ld.add_action(
        DeclareLaunchArgument("initial_positions_file", default_value=sim_initial_positions)
    )

    use_real = LaunchConfiguration("use_real_hardware")
    use_rviz = LaunchConfiguration("use_rviz")
    ld.add_action(OpaqueFunction(function=start_flight_recorder))
    ld.add_action(OpaqueFunction(function=release_direct_helpers))

    # Choose robot_description xacro + ros2_control controller YAML depending on mode
    sim_urdf_xacro = os.path.join(
        get_package_share_directory("kaixuan3_config"), "config", "kaixuan1.urdf.xacro"
    )
    sim_controllers_yaml = os.path.join(
        get_package_share_directory("kaixuan3_config"), "config", "ros2_controllers.yaml"
    )

    real_urdf_xacro = os.path.join(
        get_package_share_directory("kaixuan_can_hardware"),
        "urdf",
        "kaixuan1_ros2_control_real.urdf.xacro",
    )
    real_controllers_yaml = os.path.join(
        get_package_share_directory("kaixuan_can_hardware"),
        "config",
        "ros2_controllers.yaml",
    )

    # Build MoveIt config with the correct robot_description
    # Note: MoveItConfigsBuilder accepts LaunchConfiguration substitutions in mappings.
    sim_mappings = {
        "initial_positions_file": LaunchConfiguration("initial_positions_file"),
    }
    real_mappings = {
        "active_arm": LaunchConfiguration("active_arm"),
        "cycle_time_ns": LaunchConfiguration("cycle_time_ns"),
        "shift_time_ns": LaunchConfiguration("shift_time_ns"),
        "priority": LaunchConfiguration("priority"),
        "bus_diagnostics": LaunchConfiguration("bus_diagnostics"),
        "master0_affinity": LaunchConfiguration("master0_affinity"),
        "master1_affinity": LaunchConfiguration("master1_affinity"),
        "o6_command_timeout_ms": LaunchConfiguration("o6_command_timeout_ms"),
        "o6_max_step": LaunchConfiguration("o6_max_step"),
        "o6_imu_only": LaunchConfiguration("o6_imu_only"),
        **{f"comp{i}": LaunchConfiguration(f"comp{i}") for i in range(7)},
        **{f"left_comp{i}": LaunchConfiguration(f"left_comp{i}") for i in range(7)},
        **{
            f"right_gear_ratio{i}": LaunchConfiguration(f"right_gear_ratio{i}")
            for i in range(1, 8)
        },
        **{
            f"left_gear_ratio{i}": LaunchConfiguration(f"left_gear_ratio{i}")
            for i in range(1, 8)
        },
    }

    sim_moveit_config = (
        MoveItConfigsBuilder("kaixuan1", package_name="kaixuan3_config")
        .robot_description(file_path=sim_urdf_xacro, mappings=sim_mappings)
        .to_moveit_configs()
    )
    real_moveit_config = (
        MoveItConfigsBuilder("kaixuan1", package_name="kaixuan3_config")
        .robot_description(file_path=real_urdf_xacro, mappings=real_mappings)
        .to_moveit_configs()
    )

    # ===== TFs from SRDF virtual joints (if any) =====
    for entity in generate_static_virtual_joint_tfs_launch(sim_moveit_config).entities:
        ld.add_action(entity)

    # ===== Robot State Publisher (publishes /robot_description + TF) =====
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            respawn=True,
            parameters=[sim_moveit_config.robot_description, {"publish_frequency": 15.0}],
            condition=UnlessCondition(use_real),
        )
    )
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            respawn=True,
            parameters=[real_moveit_config.robot_description, {"publish_frequency": 15.0}],
            condition=IfCondition(use_real),
        )
    )

    # ===== ros2_control_node + controllers =====
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[sim_moveit_config.robot_description, sim_controllers_yaml],
            condition=UnlessCondition(use_real),
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[real_moveit_config.robot_description, real_controllers_yaml],
            additional_env={"KAIXUAN_SHUTDOWN_LOG": str(Path(launch_config.log_dir) / "hardware_shutdown.jsonl")},
            condition=IfCondition(use_real),
        )
    )

    # Spawn controllers
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
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration("active_arm"), "' != 'left'"])
            ),
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration("active_arm"), "' != 'right'"])
            ),
        )
    )

    # ===== MoveGroup =====
    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                sim_moveit_config.to_dict(),
                {"trajectory_execution.allowed_start_tolerance": 0.02},
                {"trajectory_execution.execution_duration_monitoring": False},
                {"trajectory_execution.allowed_execution_duration_scaling": 3.0},
            ],
            condition=UnlessCondition(use_real),
        )
    )
    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                real_moveit_config.to_dict(),
                {"trajectory_execution.allowed_start_tolerance": 0.02},
                {"trajectory_execution.execution_duration_monitoring": False},
                {"trajectory_execution.allowed_execution_duration_scaling": 3.0},
            ],
            condition=IfCondition(use_real),
        )
    )

    # ===== RViz =====
    rviz_config = os.path.join(
        get_package_share_directory("kaixuan3_config"), "config", "moveit.rviz"
    )

    ld.add_action(
        GroupAction(
            condition=IfCondition(use_rviz),
            actions=[
                Node(
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
                ),
                Node(
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
                ),
            ],
        )
    )

    return ld
