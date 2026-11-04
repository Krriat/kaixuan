"""CST ros2_control with an explicit joint selection and command watchdog."""
import json
import math
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def friction_parameters(joints, settings, identified):
    """Flatten per-joint identification for the allocation-free C++ controller."""
    result = {
        "friction_enabled": True,
        "friction_scale": float(settings.get("scale", 0.25)),
        "friction_velocity_deadband_rad_s": math.radians(float(settings.get("velocity_deadband_deg_s", 0.2))),
        "friction_velocity_full_rad_s": math.radians(float(settings.get("velocity_full_deg_s", 0.5))),
        "friction_filter_time_constant_s": float(settings.get("filter_time_constant_s", 0.02)),
        "friction_models": [], "friction_coulomb_nm": [], "friction_viscous_nm_s_per_rad": [],
        "friction_min_speed_rad_s": [], "friction_max_speed_rad_s": [],
        "friction_effort_limits_nm": [], "friction_slew_nm_s": [],
        "friction_lookup_offsets": [0], "friction_lookup_speeds_rad_s": [], "friction_lookup_torques_nm": [],
    }
    for joint in joints:
        model = identified["joints"].get(joint)
        if model is None and joint.startswith("left_"):
            raise ValueError(f"Missing identified friction model for {joint}")
        if model is not None and (not model.get("accepted_local_model") or model["model"] not in ("coulomb_viscous", "velocity_lookup")):
            raise ValueError(f"Unvalidated friction model for {joint}")
        candidate = model["coulomb_viscous_candidate"] if model else {}
        knots = model["odd_friction_velocity_knots"] if model else []
        speeds = model["speed_range_rad_s"] if model else [0.0, 0.0]
        result["friction_models"].append(model["model"] if model else "disabled")
        result["friction_coulomb_nm"].append(float(candidate.get("coulomb_nm", 0)))
        result["friction_viscous_nm_s_per_rad"].append(float(candidate.get("viscous_nm_s_per_rad", 0)))
        result["friction_min_speed_rad_s"].append(float(speeds[0]))
        result["friction_max_speed_rad_s"].append(float(speeds[1]))
        result["friction_effort_limits_nm"].append(float(settings["effort_limits_nm"][joint]) if model else 0.0)
        result["friction_slew_nm_s"].append(float(settings["slew_nm_s"][joint]) if model else 0.0)
        result["friction_lookup_speeds_rad_s"].extend(float(k["speed_rad_s"]) for k in knots)
        result["friction_lookup_torques_nm"].extend(float(k["friction_nm"]) for k in knots)
        result["friction_lookup_offsets"].append(len(result["friction_lookup_speeds_rad_s"]))
    return result


def launch_control(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    joints = [name.strip() for name in value("joints").split(",")]
    known = {f"{arm}_joint{i}" for arm in ("left", "right") for i in range(1, 8)}
    if not joints or len(set(joints)) != len(joints) or any(name not in known for name in joints):
        raise ValueError("joints must contain distinct comma-separated names, e.g. left_joint7")
    arms = {name.split("_")[0] for name in joints}
    mappings = {name: value(name) for name in (
        "cycle_time_ns", "shift_time_ns", "priority", "dc_clock_mode",
        "master0_affinity", "master1_affinity", "use_mock_hardware",
        "torque_config_file", "effort_write_timeout_ms",
        "max_velocity_rad_s", "max_displacement_deg")}
    for name in ("max_velocity_rad_s", "max_displacement_deg"):
        if mappings[name]:
            limit = float(mappings[name])
            if not math.isfinite(limit) or limit <= 0:
                raise ValueError(f"{name} must be a finite positive number")
    mappings.update(control_mode="cst", cst_joints=",".join(joints),
                    active_arm=next(iter(arms)) if len(arms) == 1 else "both")
    gravity_control = value("control_kind") == "gravity"
    mappings["stop_cst_group_on_fault"] = "true" if gravity_control else "false"
    for i in range(7):
        mappings[f"comp{i}"] = value(f"comp{i}")
        mappings[f"left_comp{i}"] = value(f"left_comp{i}")
        for arm in ("left", "right"):
            for setting in ("direction", "gear_ratio"):
                name = f"{arm}_{setting}{i + 1}"
                mappings[name] = value(name)
    hardware_share = get_package_share_directory("kaixuan_can_hardware")
    model = os.path.join(hardware_share, "urdf", "kaixuan1_ros2_control_real.urdf.xacro")
    description = {"robot_description": xacro.process_file(model, mappings=mappings).toxml()}
    controller_name = "gravity_compensation_controller" if gravity_control else "torque_controller"
    controller_type = "kaixuan_can_hardware/GravityController" if gravity_control else "kaixuan_can_hardware/TorqueController"
    parameters = {
        "controller_manager": {"ros__parameters": {
            "update_rate": 1000,
            "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            controller_name: {"type": controller_type},
        }},
        "torque_controller": {"ros__parameters": {
            "joints": joints, "command_timeout": float(value("command_timeout")),
        }},
    }
    if gravity_control:
        library = os.path.join(get_package_prefix("kaixuan_can_hardware"), "lib", "libkaixuan_gravity_controller.so")
        if not os.path.exists(library):
            raise RuntimeError("Gravity controller is not built. Install Pinocchio with scripts/install_pinocchio.sh and rebuild kaixuan_can_hardware.")
        with open(value("gravity_config_file"), encoding="utf-8") as stream:
            gravity_config = yaml.safe_load(stream)
        with open(value("torque_config_file"), encoding="utf-8") as stream:
            torque_config = yaml.safe_load(stream)["joints"]
        com = [float(v) for v in value("payload_com").split(",")]
        parameters.pop("torque_controller")
        parameters[controller_name] = {"ros__parameters": {
            **description, "joints": joints,
            "gravity_vector": gravity_config["gravity_vector"],
            "damping": [gravity_config["damping"][joint] for joint in joints],
            "effort_limits": [torque_config[joint]["effort_limit_nm"] for joint in joints],
            "gravity_scale": float(value("gravity_scale")),
            "zero_hold_duration": gravity_config["zero_hold_duration"],
            "ramp_duration": float(value("gravity_ramp_time")),
            "payload_mass": float(value("payload_mass")), "payload_com": com,
            "payload_frame": value("payload_frame"),
        }}
        friction = dict(gravity_config.get("friction_compensation", {}))
        enabled = friction.get("enabled", False)
        if not isinstance(enabled, bool):
            raise ValueError("friction_compensation.enabled must be a YAML boolean")
        if value("friction_enabled"):
            enabled = value("friction_enabled") == "true"
        if value("friction_scale"):
            friction["scale"] = float(value("friction_scale"))
        parameters[controller_name]["ros__parameters"]["friction_enabled"] = enabled
        if enabled:
            friction_file = value("friction_parameter_file") or friction.get("parameter_file", "left_arm_friction_identified.yaml")
            if not os.path.isabs(friction_file):
                friction_file = os.path.join(os.path.dirname(value("gravity_config_file")), friction_file)
            with open(friction_file, encoding="utf-8") as stream:
                identified = yaml.safe_load(stream)
            loaded_friction = friction_parameters(joints, friction, identified)
            if all(model == "disabled" for model in loaded_friction["friction_models"]):
                parameters[controller_name]["ros__parameters"]["friction_enabled"] = False
            else:
                parameters[controller_name]["ros__parameters"].update(loaded_friction)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", prefix="kaixuan_cst_", delete=False) as stream:
        yaml.safe_dump(parameters, stream)
        parameter_path = stream.name

    def cleanup(_context):
        if os.path.exists(parameter_path):
            os.unlink(parameter_path)
        return []

    manager = Node(package="controller_manager", executable="ros2_control_node",
                   parameters=[description, parameter_path], output="screen")
    spawners = [Node(package="controller_manager", executable="spawner", output="screen",
                     arguments=["joint_state_broadcaster", "torque_controller",
                                "--controller-manager-timeout", "120", "--switch-timeout", "30"])]
    if gravity_control:
        spawners = [Node(package="controller_manager", executable="spawner", output="screen",
                         arguments=names + ["--controller-manager-timeout", "120", "--switch-timeout", "30"])
                    for names in (["joint_state_broadcaster"], [controller_name, "--inactive"])]
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(target_action=manager,
                             on_exit=[EmitEvent(event=Shutdown(reason="controller_manager exited"))])),
        manager,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description], output="screen"),
        *spawners,
    ]


def generate_launch_description():
    share = get_package_share_directory("kaixuan3_config")
    with open(os.path.join(share, "config", "left_arm_calibration.json"), encoding="utf-8") as stream:
        calibration = {row["joint"]: row for row in json.load(stream)["joints"]}
    arguments = [
        DeclareLaunchArgument("joints", default_value="left_joint7",
                              description="Comma-separated joints permitted to use effort control."),
        DeclareLaunchArgument("use_mock_hardware", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("control_kind", default_value="torque", choices=["torque", "gravity"]),
        DeclareLaunchArgument("gravity_scale", default_value="1.0"),
        DeclareLaunchArgument("gravity_ramp_time", default_value="5.0"),
        DeclareLaunchArgument("friction_enabled", default_value="", choices=["", "true", "false"]),
        DeclareLaunchArgument("friction_scale", default_value="", description="Optional friction gain in [0,1]; empty uses gravity YAML."),
        DeclareLaunchArgument("friction_parameter_file", default_value="", description="Optional identification YAML override."),
        DeclareLaunchArgument("gravity_config_file", default_value=os.path.join(
            get_package_share_directory("kaixuan_can_hardware"), "config", "gravity_compensation.yaml")),
        DeclareLaunchArgument("payload_mass", default_value="0.0", description="Additional tool/load mass in kg."),
        DeclareLaunchArgument("payload_com", default_value="0,0,0", description="COM relative to payload_frame, metres."),
        DeclareLaunchArgument("payload_frame", default_value="left_ee"),
        DeclareLaunchArgument("command_timeout", default_value="0.2", description="Topic receipt timeout, seconds."),
        DeclareLaunchArgument("effort_write_timeout_ms", default_value="100"),
        DeclareLaunchArgument("max_velocity_rad_s", default_value="",
                              description="Optional CST speed limit override in rad/s; empty uses the YAML limits."),
        DeclareLaunchArgument("max_displacement_deg", default_value="",
                              description="Optional displacement from each enable position, degrees; empty uses YAML."),
        DeclareLaunchArgument("torque_config_file", default_value=os.path.join(
            get_package_share_directory("kaixuan_can_hardware"), "config", "torque_limits.yaml")),
        DeclareLaunchArgument("cycle_time_ns", default_value="1000000"),
        DeclareLaunchArgument("shift_time_ns", default_value="500000"),
        DeclareLaunchArgument("dc_clock_mode", default_value="reference", choices=["reference", "host"]),
        DeclareLaunchArgument("priority", default_value="90"),
        DeclareLaunchArgument("master0_affinity", default_value="6"),
        DeclareLaunchArgument("master1_affinity", default_value="7"),
    ]
    # Same right-arm defaults as moveit_bringup; left-arm values share its source record.
    right_offsets = (8.61927750, -115.05901903, -151.79905785, 43.58607398,
                     125.67501068, -9.17790101, -0.02567102)
    right_directions = (-1, 1, -1, -1, -1, -1, 1)
    ratios = (101, 101, 81, 81, 101, 101, 101)
    for i in range(7):
        left = calibration[f"left_joint{i + 1}"]
        defaults = {
            f"comp{i}": right_offsets[i], f"left_comp{i}": left["new_comp_deg"],
            f"right_direction{i + 1}": right_directions[i], f"left_direction{i + 1}": left["direction"],
            f"right_gear_ratio{i + 1}": ratios[i], f"left_gear_ratio{i + 1}": left["gear_ratio"],
        }
        arguments.extend(DeclareLaunchArgument(name, default_value=str(default)) for name, default in defaults.items())
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_control)])
