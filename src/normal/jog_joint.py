#!/usr/bin/env python3

import argparse
import math
import os
import shlex
import sys
from typing import Optional


def _force_udp_only_local_transport() -> None:
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")


_force_udp_only_local_transport()


def _ensure_ros_runtime_env() -> None:
    if os.environ.get("_JOG_JOINT_ROS_ENV_READY") == "1":
        return
    if os.environ.get("AMENT_PREFIX_PATH") and os.environ.get("LD_LIBRARY_PATH"):
        return

    script_path = os.path.abspath(__file__)
    quoted_args = " ".join(shlex.quote(arg) for arg in sys.argv[1:])
    command = (
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
        "source /home/niic/cxl/ros2_control/install/setup.bash >/dev/null 2>&1 && "
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && "
        f"export _JOG_JOINT_ROS_ENV_READY=1 && python3 {shlex.quote(script_path)}"
    )
    if quoted_args:
        command += f" {quoted_args}"
    os.execv("/bin/bash", ["bash", "-lc", command])


_ensure_ros_runtime_env()


def _ensure_ros_python_paths() -> None:
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        f"/opt/ros/humble/lib/{version}/site-packages",
        f"/opt/ros/humble/local/lib/{version}/dist-packages",
        f"/home/niic/cxl/ros2_control/install/lib/{version}/site-packages",
        f"/home/niic/cxl/ros2_control/install/local/lib/{version}/dist-packages",
    ]
    for path in candidates:
        if os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)


_ensure_ros_python_paths()

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS_BY_ARM = {
    "right": [
        "right_joint1",
        "right_joint2",
        "right_joint3",
        "right_joint4",
        "right_joint5",
        "right_joint6",
        "right_joint7",
    ],
    "left": [
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "left_joint7",
    ],
}

# Hard safety limit for every trajectory segment. Commands above this average
# joint speed are rejected before the controller is activated or sent a goal.
MAX_JOINT_SPEED_DEG_PER_SEC = 20.0

RESULT_CODE_TEXT = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -2: "INVALID_JOINTS",
    -3: "OLD_HEADER_TIMESTAMP",
    -4: "PATH_TOLERANCE_VIOLATED",
    -5: "GOAL_TOLERANCE_VIOLATED",
}


class JogJoint(Node):
    def __init__(self, joint_states_topic: str) -> None:
        super().__init__("jog_joint")
        self._last_joint_state: Optional[JointState] = None

        self.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_state,
            10,
        )

        self._list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self._switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg

    def wait_for_joint_state(self, timeout_s: float) -> JointState:
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)
        while rclpy.ok() and self._last_joint_state is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._last_joint_state is None:
            raise TimeoutError("No /joint_states received")
        return self._last_joint_state

    def _list_controllers(self, timeout_s: float):
        if not self._list_client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError("/controller_manager/list_controllers not available")

        future = self._list_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        response = future.result()
        if response is None:
            raise RuntimeError("Failed to call /controller_manager/list_controllers")
        return response.controller

    def get_controller_state(self, controller_name: str, timeout_s: float) -> Optional[str]:
        controllers = self._list_controllers(timeout_s)
        for state in controllers:
            if state.name == controller_name:
                return state.state
        return None

    def ensure_controller_active(self, controller_name: str, timeout_s: float, auto_activate: bool) -> None:
        state = self.get_controller_state(controller_name, timeout_s)
        if state is None:
            raise RuntimeError(f"Controller not found: {controller_name}")
        if state == "active":
            return
        if not auto_activate:
            raise RuntimeError(
                f"Controller {controller_name} is '{state}'. Activate it first or remove --no-auto-activate"
            )

        self.get_logger().info(f"Activating controller: {controller_name}")

        if not self._switch_client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError("/controller_manager/switch_controller not available")

        req = SwitchController.Request()
        req.activate_controllers = [controller_name]
        req.deactivate_controllers = []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=max(1, int(timeout_s)), nanosec=0)

        future = self._switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        response = future.result()
        if response is None or not response.ok:
            raise RuntimeError(f"Failed to activate controller: {controller_name}")

        check_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)
        while rclpy.ok() and self.get_clock().now() < check_deadline:
            if self.get_controller_state(controller_name, timeout_s=1.0) == "active":
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        raise TimeoutError(f"Controller {controller_name} did not become active in time")

    def execute_joint_goal(
        self,
        controller_name: str,
        joint_names: list[str],
        current_positions: list[float],
        target_positions: list[float],
        duration_s: float,
        timeout_s: float,
        target_joint: str,
        path_tolerance_rad: Optional[float],
        goal_tolerance_rad: Optional[float],
        goal_time_tolerance_s: float,
    ) -> None:
        action_name = f"/{controller_name}/follow_joint_trajectory"
        action_client = ActionClient(self, FollowJointTrajectory, action_name)

        if not action_client.wait_for_server(timeout_sec=timeout_s):
            raise TimeoutError(f"Action server not available: {action_name}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        p0 = JointTrajectoryPoint()
        p0.positions = list(current_positions)
        p0.time_from_start = Duration(sec=0, nanosec=0)

        sec = int(duration_s)
        nsec = int((duration_s - sec) * 1e9)
        if nsec < 0:
            nsec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = list(target_positions)
        p1.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal.trajectory.points = [p0, p1]
        goal.goal_time_tolerance = _seconds_to_duration(goal_time_tolerance_s)

        if path_tolerance_rad is not None:
            goal.path_tolerance = [
                JointTolerance(
                    name=target_joint,
                    position=path_tolerance_rad,
                    velocity=0.0,
                    acceleration=0.0,
                )
            ]
        if goal_tolerance_rad is not None:
            goal.goal_tolerance = [
                JointTolerance(
                    name=target_joint,
                    position=goal_tolerance_rad,
                    velocity=0.0,
                    acceleration=0.0,
                )
            ]

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_s + timeout_s)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise TimeoutError("No result received from FollowJointTrajectory")

        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            code_name = RESULT_CODE_TEXT.get(result.error_code, f"UNKNOWN({result.error_code})")
            raise RuntimeError(f"Trajectory failed: {code_name} - {result.error_string}")


def _seconds_to_duration(seconds: float) -> Duration:
    total_nanoseconds = max(0, int(round(seconds * 1e9)))
    sec = total_nanoseconds // 1_000_000_000
    nanosec = total_nanoseconds % 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def _chunk_targets(start: float, final: float, max_step_rad: Optional[float]) -> list[float]:
    if max_step_rad is None or max_step_rad <= 0.0:
        return [final]

    delta = final - start
    if abs(delta) <= max_step_rad:
        return [final]

    steps = max(1, int(math.ceil(abs(delta) / max_step_rad)))
    return [start + delta * index / steps for index in range(1, steps + 1)]


def _segment_speed_deg_per_sec(start_rad: float, target_rad: float, duration_s: float) -> float:
    return abs(math.degrees(target_rad - start_rad)) / duration_s


def _resolve_delta_degrees(
    current_position_rad: float,
    absolute_degrees: Optional[float],
    delta_degrees: Optional[float],
) -> float:
    """Convert both command styles to one feedback-relative move."""
    if absolute_degrees is not None:
        return absolute_degrees - math.degrees(current_position_rad)
    if delta_degrees is None:
        raise ValueError("Either absolute_degrees or delta_degrees is required")
    return delta_degrees


def infer_arm(joint_name: str) -> str:
    if joint_name.startswith("right_"):
        return "right"
    if joint_name.startswith("left_"):
        return "left"
    raise ValueError(f"Joint must start with right_ or left_: {joint_name}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Move one joint angle via JointTrajectoryController while holding the other 6 joints at current values"
    )
    parser.add_argument("joint", help="Joint name, e.g. right_joint1 or left_joint3")

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--degrees", type=float, help="Absolute target angle in degrees")
    target_group.add_argument("--delta-degrees", type=float, help="Relative delta angle in degrees")

    parser.add_argument("--duration", type=float, default=3.0, help="Motion duration in seconds")
    parser.add_argument("--timeout", type=float, default=8.0, help="Service/action timeout in seconds")
    parser.add_argument(
        "--goal-time-tolerance",
        type=float,
        default=None,
        help="Extra time in seconds allowed after trajectory end before declaring GOAL_TOLERANCE_VIOLATED",
    )
    parser.add_argument(
        "--path-tolerance-degrees",
        type=float,
        default=None,
        help="Override controller path tolerance for the target joint only, in degrees",
    )
    parser.add_argument(
        "--goal-tolerance-degrees",
        type=float,
        default=None,
        help="Override controller goal tolerance for the target joint only, in degrees",
    )
    parser.add_argument(
        "--max-step-degrees",
        type=float,
        default=None,
        help="Split the requested move into smaller increments of at most this size in degrees",
    )
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--no-auto-activate", action="store_true", help="Do not auto-activate controller")

    args = parser.parse_args()

    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be a finite value > 0")

    requested_degrees = args.degrees if args.degrees is not None else args.delta_degrees
    if requested_degrees is None or not math.isfinite(requested_degrees):
        raise SystemExit("The requested angle must be finite")

    if args.goal_time_tolerance is not None and args.goal_time_tolerance < 0.0:
        raise SystemExit("--goal-time-tolerance must be >= 0")

    if args.max_step_degrees is not None and args.max_step_degrees <= 0.0:
        raise SystemExit("--max-step-degrees must be > 0")

    arm = infer_arm(args.joint)
    joint_names = JOINTS_BY_ARM[arm]
    if args.joint not in joint_names:
        raise SystemExit(f"Joint {args.joint} is not part of the {arm} arm controller")

    controller_name = f"{arm}_arm_controller"

    rclpy.init()
    node = JogJoint(joint_states_topic=args.joint_states_topic)

    try:
        js = node.wait_for_joint_state(timeout_s=args.timeout)
        current_map = {name: pos for name, pos in zip(js.name, js.position)}

        missing = [name for name in joint_names if name not in current_map]
        if missing:
            raise RuntimeError(f"Missing joints in /joint_states: {missing}")

        current_positions = [current_map[name] for name in joint_names]
        target_positions = list(current_positions)

        idx = joint_names.index(args.joint)
        if not math.isfinite(current_positions[idx]):
            raise RuntimeError(f"Invalid position feedback for {args.joint}: value is not finite")
        current_deg = math.degrees(current_positions[idx])
        command_delta_deg = _resolve_delta_degrees(
            current_position_rad=current_positions[idx],
            absolute_degrees=args.degrees,
            delta_degrees=args.delta_degrees,
        )
        target_positions[idx] = current_positions[idx] + math.radians(command_delta_deg)
        if args.degrees is not None:
            mode_text = (
                f"absolute {args.degrees:.3f} deg converted to "
                f"delta {command_delta_deg:+.3f} deg from live feedback"
            )
        else:
            mode_text = f"delta {command_delta_deg:+.3f} deg"

        final_target_deg = math.degrees(target_positions[idx])
        goal_time_tolerance_s = args.goal_time_tolerance if args.goal_time_tolerance is not None else max(1.0, args.duration)
        path_tolerance_rad = None if args.path_tolerance_degrees is None else math.radians(args.path_tolerance_degrees)
        goal_tolerance_rad = None if args.goal_tolerance_degrees is None else math.radians(args.goal_tolerance_degrees)
        max_step_rad = None if args.max_step_degrees is None else math.radians(args.max_step_degrees)

        segment_targets = _chunk_targets(current_positions[idx], target_positions[idx], max_step_rad)
        segment_starts = [current_positions[idx]] + segment_targets[:-1]
        segment_speeds = [
            _segment_speed_deg_per_sec(start, target, args.duration)
            for start, target in zip(segment_starts, segment_targets)
        ]
        max_commanded_speed = max(segment_speeds, default=0.0)

        if max_commanded_speed > MAX_JOINT_SPEED_DEG_PER_SEC:
            minimum_duration = max(
                abs(math.degrees(target - start)) / MAX_JOINT_SPEED_DEG_PER_SEC
                for start, target in zip(segment_starts, segment_targets)
            )
            raise RuntimeError(
                f"Safety limit exceeded: requested speed={max_commanded_speed:.3f} deg/s, "
                f"limit={MAX_JOINT_SPEED_DEG_PER_SEC:.3f} deg/s. "
                f"Use --duration {minimum_duration:.3f} or longer. No motion command was sent."
            )

        node.ensure_controller_active(
            controller_name=controller_name,
            timeout_s=args.timeout,
            auto_activate=not args.no_auto_activate,
        )

        node.get_logger().info(
            f"Sending {args.joint} -> {mode_text} via {controller_name}, current={current_deg:.3f} deg, "
            f"target={final_target_deg:.3f} deg, duration={args.duration:.2f}s, "
            f"speed={max_commanded_speed:.3f} deg/s"
        )

        if len(segment_targets) > 1:
            node.get_logger().info(
                f"Splitting move into {len(segment_targets)} segments, max step={args.max_step_degrees:.3f} deg"
            )

        for segment_index, (segment_start, segment_target) in enumerate(zip(segment_starts, segment_targets), start=1):
            segment_current_positions = list(current_positions)
            segment_target_positions = list(current_positions)
            segment_current_positions[idx] = segment_start
            segment_target_positions[idx] = segment_target

            if len(segment_targets) > 1:
                node.get_logger().info(
                    f"Segment {segment_index}/{len(segment_targets)}: {math.degrees(segment_start):.3f} -> {math.degrees(segment_target):.3f} deg"
                )

            node.execute_joint_goal(
                controller_name=controller_name,
                joint_names=joint_names,
                current_positions=segment_current_positions,
                target_positions=segment_target_positions,
                duration_s=args.duration,
                timeout_s=args.timeout,
                target_joint=args.joint,
                path_tolerance_rad=path_tolerance_rad,
                goal_tolerance_rad=goal_tolerance_rad,
                goal_time_tolerance_s=goal_time_tolerance_s,
            )
            current_positions[idx] = segment_target

        refreshed_js = node.wait_for_joint_state(timeout_s=args.timeout)
        refreshed_map = {name: pos for name, pos in zip(refreshed_js.name, refreshed_js.position)}
        actual_deg = math.degrees(refreshed_map[args.joint])
        new_deg = math.degrees(target_positions[idx])
        node.get_logger().info(f"Actual {args.joint} after command = {actual_deg:.3f} deg")
        node.get_logger().info(f"Done. Target {args.joint} = {new_deg:.3f} deg")
    except Exception as exc:
        if 'node' in locals() and args.joint in JOINTS_BY_ARM.get(arm, []):
            try:
                latest_js = node.wait_for_joint_state(timeout_s=1.0)
                latest_map = {name: pos for name, pos in zip(latest_js.name, latest_js.position)}
                if args.joint in latest_map:
                    node.get_logger().error(
                        f"Current measured {args.joint} = {math.degrees(latest_map[args.joint]):.3f} deg"
                    )
            except Exception:
                pass
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
