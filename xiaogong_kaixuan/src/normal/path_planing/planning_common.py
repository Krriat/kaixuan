#!/usr/bin/env python3
"""ROS communication, execution and output shared by the seven examples.

Lengths are metres, angles are radians, and all pose targets use base_link.
"""

import argparse
import copy
import csv
import json
import math
import os
import time
import uuid
from datetime import datetime
from pathlib import Path

os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, Vector3
from moveit_msgs.msg import (
    Constraints, DisplayTrajectory, JointConstraint, MoveItErrorCodes,
    OrientationConstraint, PositionConstraint, RobotState,
)
from moveit_msgs.srv import (
    GetCartesianPath, GetMotionPlan, GetPlannerParams, GetPositionFK,
    QueryPlannerInterfaces, SetPlannerParams,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from unique_identifier_msgs.msg import UUID
from visualization_msgs.msg import Marker, MarkerArray

GROUP = "left_arm"
TIP = "left_link7"
FRAME = "base_link"
JOINTS = [f"left_joint{i}" for i in range(1, 8)]
OMPL_IDS = {"rrtstar": "RRTstar", "rrtconnect": "RRTConnect", "prm": "PRM", "est": "EST"}
COLORS = [(1.0, .2, .2), (.2, 1.0, .2), (.2, .5, 1.0), (1.0, .6, .1),
          (1.0, .2, 1.0), (.1, 1.0, 1.0), (1.0, 1.0, .2)]


def duration_seconds(value):
    return value.sec + value.nanosec * 1e-9


def set_duration(value, seconds):
    ns = round(seconds * 1e9)
    value.sec, value.nanosec = divmod(ns, 1_000_000_000)


def error_text(code):
    names = [name for name in dir(MoveItErrorCodes)
             if name.isupper() and getattr(MoveItErrorCodes, name) == code]
    return f"{names[0] if names else 'UNKNOWN'} ({code})"


def ros_environment_text():
    domain = os.environ.get("ROS_DOMAIN_ID", "0 (unset; ROS default)")
    return (f"ROS_DOMAIN_ID={domain}; RMW={get_rmw_implementation_identifier()}; "
            f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '0')}")


def state_positions(state):
    return dict(zip(state.joint_state.name, state.joint_state.position))


def with_positions(state, names, positions):
    result = copy.deepcopy(state)
    values = state_positions(result)
    values.update(zip(names, positions))
    result.joint_state.name = list(values)
    result.joint_state.position = list(values.values())
    result.joint_state.velocity = [0.0] * len(values)
    result.joint_state.effort = []
    return result


def joint_goal(values):
    goal = Constraints()
    for name, value in zip(JOINTS, values):
        goal.joint_constraints.append(JointConstraint(
            joint_name=name, position=float(value),
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
    return goal


def pose_goal(pose):
    goal = Constraints()
    position = PositionConstraint(link_name=TIP, weight=1.0)
    position.header.frame_id = FRAME
    position.constraint_region.primitives = [SolidPrimitive(
        type=SolidPrimitive.SPHERE, dimensions=[0.0001])]
    position.constraint_region.primitive_poses = [copy.deepcopy(pose)]
    orientation = OrientationConstraint(
        link_name=TIP, orientation=copy.deepcopy(pose.orientation),
        absolute_x_axis_tolerance=0.001, absolute_y_axis_tolerance=0.001,
        absolute_z_axis_tolerance=0.001, weight=1.0)
    orientation.header.frame_id = FRAME
    goal.position_constraints = [position]
    goal.orientation_constraints = [orientation]
    return goal


def validate_trajectory(trajectory):
    """Never allow another arm, a partial joint set, or invalid timing to execute."""
    jt = trajectory.joint_trajectory
    if len(jt.joint_names) != 7 or set(jt.joint_names) != set(JOINTS):
        raise RuntimeError(f"Expected only seven left joints, got {jt.joint_names}")
    if trajectory.multi_dof_joint_trajectory.points:
        raise RuntimeError("Unexpected multi-DOF trajectory")
    if len(jt.points) < 2:
        raise RuntimeError("Trajectory contains fewer than two points")
    previous = -1.0
    for point in jt.points:
        if len(point.positions) != 7:
            raise RuntimeError("Invalid trajectory position count")
        for values in (point.positions, point.velocities, point.accelerations):
            if values and (len(values) != 7 or not all(math.isfinite(v) for v in values)):
                raise RuntimeError("Invalid trajectory values")
        seconds = duration_seconds(point.time_from_start)
        if seconds < 0.0 or seconds <= previous:
            raise RuntimeError("Trajectory timestamps must be strictly increasing")
        previous = seconds
    return previous


def stretch_trajectory(trajectory, factor):
    """Preserve the server's timing profile, scaling t, v and a consistently."""
    for point in trajectory.joint_trajectory.points:
        set_duration(point.time_from_start, duration_seconds(point.time_from_start) * factor)
        point.velocities = [v / factor for v in point.velocities]
        point.accelerations = [a / (factor * factor) for a in point.accelerations]


def hardware_execution_error(statuses):
    """Decode this workspace's existing EtherCAT readiness/drive diagnostics."""
    bus = statuses.get("ethercat/master1")
    if bus is None:
        return "Missing left-arm EtherCAT diagnostics: ethercat/master1"
    if bus.get("dc_follower_failed") == "1":
        return ("Left-arm EtherCAT DC follower failed "
                f"(dc_follower_error={bus.get('dc_follower_error', '?')}); drives disabled. "
                "Restart bringup to requalify clock synchronization before executing.")
    if bus.get("dc_reset_failed") == "1":
        return "Left-arm EtherCAT DC qualification failed; drives disabled"
    if bus.get("wc_state") != "2":
        return "Left-arm EtherCAT working counter is incomplete"
    if bus.get("startup_ready") != "1":
        return "Left-arm EtherCAT startup qualification is not ready; drives are not enabled"
    for name in JOINTS:
        axis = statuses.get(name)
        if axis is None:
            return f"Missing drive diagnostics for {name}"
        try:
            status = int(axis["status_word"], 0)
            error = int(axis["error_code"], 0)
        except (KeyError, ValueError):
            return f"Invalid drive diagnostics for {name}"
        if error:
            return f"{name} reports drive error_code={error}"
        if status & 0x006F != 0x0027:
            return f"{name} is not operation-enabled: status_word=0x{status:04x}, control_word={axis.get('control_word', '?')}"
    return None


class LeftArmPlanner(Node):
    def __init__(self):
        super().__init__("left_arm_planning_examples")
        self.latest = None
        self.create_subscription(JointState, "/joint_states", self._on_state, qos_profile_sensor_data)
        self.hardware_statuses = {}
        self.hardware_received_at = 0.0
        self.real_ethercat_hardware = None
        self.create_subscription(DiagnosticArray, "/kaixuan/hardware_diagnostics",
                                 self._on_hardware_diagnostics, qos_profile_sensor_data)
        self.hardware_client = self.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.query_client = self.create_client(QueryPlannerInterfaces, "/query_planner_interface")
        self.planner_interfaces = None
        self.set_params_client = self.create_client(SetPlannerParams, "/set_planner_params")
        self.get_params_client = self.create_client(GetPlannerParams, "/get_planner_params")
        self.configured_ompl = set()
        self.execute_action = "/left_arm_controller/follow_joint_trajectory"
        self.execute_client = ActionClient(self, FollowJointTrajectory, self.execute_action)
        self.cancel_client = self.create_client(CancelGoal, self.execute_action + "/_action/cancel_goal")
        self.active_goal = None
        self.pending_goal_id = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.display_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/left_arm_planning_paths", qos)
        self.markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])

    def _on_state(self, message):
        if (len(message.name) == len(message.position)
                and set(JOINTS).issubset(message.name)
                and all(math.isfinite(q) for q in message.position)):
            self.latest = message

    def _on_hardware_diagnostics(self, message):
        self.hardware_statuses = {status.name: {v.key: v.value for v in status.values}
                                  for status in message.status}
        self.hardware_received_at = time.monotonic()

    def check_hardware_ready(self):
        if self.real_ethercat_hardware is None:
            # DDS discovery can take longer than three seconds on a new SSH terminal.
            if not self.hardware_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f"Waiting for {self.hardware_client.srv_name}; {ros_environment_text()}")
                if not self.hardware_client.wait_for_service(timeout_sec=14.0):
                    raise RuntimeError(
                        f"Service unavailable: {self.hardware_client.srv_name} after 15s; "
                        f"{ros_environment_text()}. Source this workspace and use the same "
                        "ROS_DOMAIN_ID and RMW as the bringup terminal; execution blocked.")
            response = self.wait_result(self.hardware_client.call_async(ListHardwareComponents.Request()),
                                        3.0, f"List hardware components ({ros_environment_text()})")
            self.real_ethercat_hardware = any(
                c.name == "KaixuanCanSystem" or c.class_type == "kaixuan_can_hardware/KaixuanCanSystem"
                for c in response.component)
        if not self.real_ethercat_hardware:
            return
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and (not self.hardware_received_at or time.monotonic() - self.hardware_received_at > 1.0):
            if time.monotonic() >= deadline:
                raise RuntimeError("No fresh EtherCAT hardware diagnostics; execution blocked")
            rclpy.spin_once(self, timeout_sec=0.1)
        reason = hardware_execution_error(self.hardware_statuses)
        if reason:
            raise RuntimeError(reason)

    def wait_result(self, future, timeout, label):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise RuntimeError(f"{label} timed out after {timeout:.1f}s")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def call(self, client, request, timeout=15.0):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {client.srv_name}; MoveIt may still be loading meshes")
            if not client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError(f"Service unavailable: {client.srv_name}; start MoveIt first")
        return self.wait_result(client.call_async(request), timeout, client.srv_name)

    def require_planner(self, pipeline, planner_id):
        # MoveIt can silently fall back to RRTConnect for an unregistered ID.
        # Refuse that case so the comparison always runs the requested algorithm.
        if self.planner_interfaces is None:
            self.planner_interfaces = self.call(self.query_client, QueryPlannerInterfaces.Request()).planner_interfaces
        required = f"{GROUP}[{planner_id}]" if pipeline == "ompl" else planner_id
        if not any(p.pipeline_id == pipeline and required in p.planner_ids for p in self.planner_interfaces):
            raise RuntimeError(f"Planner {pipeline}/{required} is not registered; rebuild and source this workspace")

    def configure_ompl(self, planner, parameters):
        if planner in self.configured_ompl:
            return
        values = {key: str(value) for key, value in parameters.items()}
        request = SetPlannerParams.Request()
        request.pipeline_id = "ompl"
        request.group = GROUP
        request.planner_config = OMPL_IDS[planner]
        request.replace = False
        request.params.keys = list(values)
        request.params.values = list(values.values())
        self.call(self.set_params_client, request)
        check = GetPlannerParams.Request(
            pipeline_id="ompl", group=GROUP, planner_config=OMPL_IDS[planner])
        response = self.call(self.get_params_client, check)
        actual = dict(zip(response.params.keys, response.params.values))
        if any(actual.get(key) != value for key, value in values.items()):
            raise RuntimeError(f"OMPL parameter verification failed for {planner}: {actual}")
        self.configured_ompl.add(planner)
        self.get_logger().info(f"{OMPL_IDS[planner]} parameters: {values}")

    def snapshot(self, timeout=8.0):
        """Wait for fresh stationary feedback before requesting a zero-velocity start."""
        self.latest = None
        reference = None
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            msg = self.latest
            if msg is None:
                continue
            q = dict(zip(msg.name, msg.position))
            velocities = dict(zip(msg.name, msg.velocity))
            if any(abs(velocities.get(name, 0.0)) > 0.02 for name in JOINTS):
                reference = None
                continue
            now = time.monotonic()
            if reference is None:
                reference = (now, q, msg)
            elif now - reference[0] >= 0.2 and msg is not reference[2]:
                if max(abs(q[n] - reference[1][n]) for n in JOINTS) <= 0.002:
                    state = RobotState(is_diff=True)
                    state.joint_state = copy.deepcopy(msg)
                    state.joint_state.velocity = [0.0] * len(msg.name)
                    state.joint_state.effort = []
                    return state
                reference = (now, q, msg)
        raise RuntimeError("No fresh stationary left-arm /joint_states feedback")

    def fk(self, state):
        request = GetPositionFK.Request()
        request.header.frame_id = FRAME
        request.fk_link_names = [TIP]
        request.robot_state = state
        response = self.call(self.fk_client, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
            raise RuntimeError(f"FK failed: {error_text(response.error_code.val)}")
        return response.pose_stamped[0].pose

    def plan_motion(self, start, goal, planner, args, interim=None):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = GROUP
        motion.start_state = copy.deepcopy(start)
        motion.goal_constraints = [goal]
        motion.num_planning_attempts = 1
        motion.allowed_planning_time = args.planning_time
        motion.max_velocity_scaling_factor = args.velocity_scale
        motion.max_acceleration_scaling_factor = args.acceleration_scale
        motion.workspace_parameters.header.frame_id = FRAME
        motion.workspace_parameters.min_corner = Vector3(x=-2.0, y=-2.0, z=-2.0)
        motion.workspace_parameters.max_corner = Vector3(x=2.0, y=2.0, z=2.0)
        if planner in OMPL_IDS:
            motion.pipeline_id = "ompl"
            motion.planner_id = OMPL_IDS[planner]
        else:
            motion.pipeline_id = "pilz_industrial_motion_planner"
            motion.planner_id = planner.upper()
            if planner == "circ":
                # Three points define the circular arc: start, interim, goal.
                motion.path_constraints.name = "interim"
                position = PositionConstraint(link_name=TIP, weight=1.0)
                position.header.frame_id = FRAME
                position.constraint_region.primitive_poses = [interim]
                motion.path_constraints.position_constraints = [position]
        self.require_planner(motion.pipeline_id, motion.planner_id)
        if planner in OMPL_IDS:
            self.configure_ompl(planner, args.ompl_parameters[planner])
        response = self.call(self.plan_client, request, args.planning_time + 15.0)
        result = response.motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{motion.pipeline_id}/{motion.planner_id}: "
                               f"{error_text(result.error_code.val)}")
        validate_trajectory(result.trajectory)
        return result.trajectory, float(result.planning_time), None

    def plan_cartesian(self, start, waypoints, args):
        request = GetCartesianPath.Request()
        request.header.frame_id = FRAME
        request.start_state = copy.deepcopy(start)
        request.group_name = GROUP
        request.link_name = TIP
        request.waypoints = waypoints
        request.max_step = args.max_step
        request.jump_threshold = args.jump_threshold
        request.revolute_jump_threshold = math.radians(args.max_joint_jump_deg)
        request.avoid_collisions = True
        # Some Humble installations do not expose scaling in GetCartesianPath.
        has_scaling = hasattr(request, "max_velocity_scaling_factor")
        if has_scaling:
            request.max_velocity_scaling_factor = args.velocity_scale
            request.max_acceleration_scaling_factor = args.acceleration_scale
        begin = time.monotonic()
        response = self.call(self.cartesian_client, request, args.planning_time + 15.0)
        elapsed = time.monotonic() - begin
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Cartesian: {error_text(response.error_code.val)}")
        if not math.isfinite(response.fraction) or response.fraction < 1.0 - 1e-6:
            raise RuntimeError(f"Cartesian path incomplete: fraction={response.fraction:.6f}")
        trajectory = response.solution
        validate_trajectory(trajectory)
        if not has_scaling:
            factor = max(1.0 / args.velocity_scale, 1.0 / math.sqrt(args.acceleration_scale))
            stretch_trajectory(trajectory, factor)
            self.get_logger().info(f"Cartesian timing stretched by {factor:.3f} (Humble API)")
        # Check absolute joint jumps ourselves too: older servers ignore the field.
        points = trajectory.joint_trajectory.points
        if any(max(abs(a - b) for a, b in zip(p.positions, q.positions)) > math.radians(args.max_joint_jump_deg)
               for p, q in zip(points, points[1:])):
            raise RuntimeError(f"Cartesian trajectory contains a joint jump over {args.max_joint_jump_deg} degrees")
        return trajectory, elapsed, response.fraction

    def preview_and_save(self, start, trajectory, label, target, directory, color_index):
        duration = validate_trajectory(trajectory)
        jt = trajectory.joint_trajectory
        # Sample up to 100 FK poses for display and approximate Cartesian metrics.
        indices = sorted({round(i * (len(jt.points) - 1) / min(99, len(jt.points) - 1))
                          for i in range(min(100, len(jt.points)))})
        poses = {}
        for index in indices:
            point = jt.points[index]
            poses[index] = self.fk(with_positions(start, jt.joint_names, point.positions))
        xyz = [tuple(getattr(poses[i].position, axis) for axis in "xyz") for i in indices]
        path_length = sum(math.dist(a, b) for a, b in zip(xyz, xyz[1:]))
        chord = math.dist(xyz[0], xyz[-1])
        end_error = math.dist(xyz[-1], (target.position.x, target.position.y, target.position.z))
        joint_length = sum(math.dist(p.positions, q.positions) for p, q in zip(jt.points, jt.points[1:]))
        with (directory / f"{label}.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", *jt.joint_names,
                             *[f"{n}_vel" for n in jt.joint_names],
                             *[f"{n}_acc" for n in jt.joint_names], "ee_x", "ee_y", "ee_z"])
            for index, point in enumerate(jt.points):
                pose = poses.get(index)
                writer.writerow([duration_seconds(point.time_from_start), *point.positions,
                                 *(list(point.velocities) or [""] * 7),
                                 *(list(point.accelerations) or [""] * 7),
                                 *([getattr(pose.position, a) for a in "xyz"] if pose else [""] * 3)])
        display = DisplayTrajectory(model_id="kaixuan1", trajectory_start=start, trajectory=[trajectory])
        self.display_pub.publish(display)
        marker = Marker(ns="left_arm_examples", id=len(self.markers.markers),
                        type=Marker.LINE_STRIP, action=Marker.ADD)
        marker.header.frame_id = FRAME
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.002
        marker.color.r, marker.color.g, marker.color.b = COLORS[color_index % len(COLORS)]
        marker.color.a = 1.0
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in xyz]
        self.markers.markers.append(marker)
        self.marker_pub.publish(self.markers)
        return {"points": len(jt.points), "duration_s": duration,
                "joint_length_rad": joint_length, "ee_length_mm": path_length * 1000,
                "ee_chord_mm": chord * 1000, "goal_error_mm": end_error * 1000,
                "max_velocity_rad_s": max((abs(v) for p in jt.points for v in p.velocities), default=0.0),
                "max_acceleration_rad_s2": max((abs(a) for p in jt.points for a in p.accelerations), default=0.0)}

    def cancel_active(self):
        if self.active_goal is not None:
            future = self.active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            self.active_goal = None
        elif self.pending_goal_id is not None and self.cancel_client.wait_for_service(timeout_sec=1.0):
            request = CancelGoal.Request()
            request.goal_info.goal_id = self.pending_goal_id
            future = self.cancel_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.pending_goal_id = None

    def execute(self, trajectory, planned_start):
        self.check_hardware_ready()
        duration = validate_trajectory(trajectory)
        actual = state_positions(self.snapshot())
        first = dict(zip(trajectory.joint_trajectory.joint_names,
                         trajectory.joint_trajectory.points[0].positions))
        if max(abs(actual[n] - first[n]) for n in JOINTS) > 0.01:
            raise RuntimeError("Left arm moved after planning; rerun from its current position")
        before = state_positions(planned_start)
        if any(abs(actual[n] - q) > 0.01 for n, q in before.items()
               if n not in JOINTS and n in actual):
            raise RuntimeError("Other robot joints moved after planning; replan against the current scene")
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"{self.execute_action} is unavailable")
        trajectory = copy.deepcopy(trajectory)
        trajectory.joint_trajectory.header.stamp.sec = 0
        trajectory.joint_trajectory.header.stamp.nanosec = 0
        goal = FollowJointTrajectory.Goal(trajectory=trajectory.joint_trajectory)
        self.get_logger().info(f"Sending left-arm trajectory: planned duration={duration:.3f}s")
        next_feedback_log = 0.0

        def on_feedback(message):
            nonlocal next_feedback_log
            feedback = message.feedback
            now = time.monotonic()
            if now < next_feedback_log or len(feedback.actual.positions) != len(feedback.joint_names):
                return
            positions = dict(zip(feedback.joint_names, feedback.actual.positions))
            if not set(JOINTS).issubset(positions):
                return
            movement = max(abs(positions[name] - first[name]) for name in JOINTS)
            error = max((abs(value) for value in feedback.error.positions), default=0.0)
            self.get_logger().info(
                f"Execution feedback: t={duration_seconds(feedback.desired.time_from_start):.2f}/{duration:.2f}s, "
                f"max_actual_change={math.degrees(movement):.3f}deg, max_tracking_error={math.degrees(error):.3f}deg")
            next_feedback_log = now + 1.0

        # Remember the UUID even before acceptance, for targeted cancellation.
        self.pending_goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        try:
            future = self.execute_client.send_goal_async(goal, goal_uuid=self.pending_goal_id, feedback_callback=on_feedback)
            handle = self.wait_result(future, 10.0, "Execute goal acceptance")
            if not handle.accepted:
                self.pending_goal_id = None
                raise RuntimeError("Left-arm execution rejected")
            self.active_goal = handle
            self.get_logger().info("Left-arm controller accepted the trajectory")
            result_future = handle.get_result_async()
            deadline = time.monotonic() + max(30.0, duration * 2 + 10)
            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.monotonic() >= deadline:
                    raise RuntimeError("Execute trajectory timed out")
                if self.real_ethercat_hardware:
                    if time.monotonic() - self.hardware_received_at > 2.0:
                        raise RuntimeError("EtherCAT hardware diagnostics became stale during execution")
                    reason = hardware_execution_error(self.hardware_statuses)
                    if reason:
                        raise RuntimeError(reason)
            wrapped = result_future.result()
            if wrapped is None:
                raise RuntimeError("Execute trajectory returned no result")
            if (wrapped.status != GoalStatus.STATUS_SUCCEEDED
                    or wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                raise RuntimeError(f"Execution failed: controller code={wrapped.result.error_code}, "
                                   f"{wrapped.result.error_string}")
            self.get_logger().info("Left-arm execution succeeded")
        except BaseException:
            self.cancel_active()
            raise
        finally:
            self.active_goal = None
            self.pending_goal_id = None


def parse_args(planner, parameters, argv=None):
    """Validate only the parameters supplied by the selected Python example."""
    parser = argparse.ArgumentParser(description=f"Left-arm {planner}; edit parameters in this Python file")
    parser.add_argument("--show-config", action="store_true", help="Print this file's parameters and exit without motion")
    parser.add_argument("--plan-only", action="store_true", help="Plan without execution for this run")
    parser.add_argument("--output-dir", type=Path)
    cli = parser.parse_args(argv)
    if planner not in {*OMPL_IDS, "cartesian", "lin", "circ"}:
        parser.error(f"Unsupported planner: {planner}")
    args = argparse.Namespace(**copy.deepcopy(parameters))
    required = {"velocity_scale", "acceleration_scale", "planning_time", "repeat", "pause", "execute", "return_to_start"}
    if not required.issubset(parameters):
        parser.error(f"Missing Python parameters: {sorted(required - parameters.keys())}")
    for name in ("execute", "return_to_start"):
        if type(getattr(args, name)) is not bool:
            parser.error(f"{name} must be True or False")
    for name in ("planning_time", "velocity_scale", "acceleration_scale"):
        value = getattr(args, name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")
        setattr(args, name, float(value))
    if args.velocity_scale > 1 or args.acceleration_scale > 1:
        parser.error("Scaling factors must be in (0, 1]")
    if type(args.repeat) is not int or args.repeat < 1:
        parser.error("REPEAT must be a positive integer")
    if type(args.pause) not in (int, float) or not math.isfinite(args.pause) or args.pause < 0:
        parser.error("PAUSE_S must be finite and nonnegative")
    args.pause = float(args.pause)

    def vector(value, size, name):
        if (not isinstance(value, (list, tuple)) or len(value) != size
                or not all(type(v) in (int, float) and math.isfinite(v) for v in value)):
            parser.error(f"{name} must contain {size} finite numbers")

    if planner in OMPL_IDS:
        pose_inputs = {"delta", "target_position_m", "target_rpy_deg"}.intersection(parameters)
        if pose_inputs:
            parser.error(f"OMPL accepts only TARGET_JOINTS_DEG; remove pose inputs: {sorted(pose_inputs)}")
        if getattr(args, "joint_goal_deg", None) is None and not cli.show_config:
            parser.error("Fill TARGET_JOINTS_DEG with seven absolute joint angles in degrees before running")

    # Non-applicable fields are None; no other example's parameter values are loaded.
    for name, size in (("delta", 3), ("target_position_m", 3), ("target_rpy_deg", 3),
                       ("joint_goal_deg", 7), ("arc_offset", 3), ("arc_position_m", 3)):
        value = getattr(args, name, None)
        if value is not None:
            vector(value, size, name)
        setattr(args, name, value)
    if planner == "cartesian":
        args.waypoint_positions_m = getattr(args, "waypoint_positions_m", None)
        args.waypoint_offsets_m = getattr(args, "waypoint_offsets_m", None)
        points = args.waypoint_positions_m if args.waypoint_positions_m is not None else args.waypoint_offsets_m
        if not isinstance(points, (list, tuple)) or not points:
            parser.error("Cartesian needs a nonempty WAYPOINT_OFFSETS_M or WAYPOINT_POSITIONS_M list")
        for point in points:
            vector(point, 3, "Cartesian waypoint")
        for name in ("max_step", "max_joint_jump_deg"):
            value = getattr(args, name, None)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                parser.error(f"{name} must be finite and positive")
            setattr(args, name, float(value))
        value = getattr(args, "jump_threshold", None)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            parser.error("RELATIVE_JUMP_THRESHOLD must be finite and nonnegative")
        args.jump_threshold = float(value)
    elif planner not in OMPL_IDS and args.target_position_m is None and args.delta is None:
        parser.error("Specify TARGET_POSITION_M or TARGET_OFFSET_M in this Python file")
    if planner == "circ" and args.arc_position_m is None and args.arc_offset is None:
        parser.error("CIRC needs VIA_POSITION_M or VIA_OFFSET_M")

    args.ompl_parameters = {}
    if planner in OMPL_IDS:
        allowed = {"rrtstar": {"range", "goal_bias", "delay_collision_checking"},
                   "rrtconnect": {"range"}, "prm": {"max_nearest_neighbors"},
                   "est": {"range", "goal_bias"}}[planner]
        tuning = getattr(args, "planner_parameters", None)
        if not isinstance(tuning, dict) or set(tuning) != allowed:
            parser.error(f"PLANNER_PARAMETERS for {planner} must contain {sorted(allowed)}")
        for name, value in tuning.items():
            if type(value) not in (int, float) or not math.isfinite(value):
                parser.error(f"PLANNER_PARAMETERS.{name} must be finite")
            if name == "range" and value < 0:
                parser.error("range must be nonnegative")
            if name == "goal_bias" and not 0 <= value <= 1:
                parser.error("goal_bias must be in [0, 1]")
            if name == "max_nearest_neighbors" and (type(value) is not int or value < 1):
                parser.error("max_nearest_neighbors must be a positive integer")
            if name == "delay_collision_checking" and (type(value) is not int or value not in (0, 1)):
                parser.error("delay_collision_checking must be 0 or 1")
        args.ompl_parameters[planner] = tuning
    args.execute = args.execute and not cli.plan_only
    args.return_to_start = args.return_to_start and args.execute
    if args.execute and args.repeat > 1 and not args.return_to_start:
        parser.error("Repeated execution requires RETURN_TO_START = True")
    if args.return_to_start and "rrtconnect" not in args.ompl_parameters:
        value = getattr(args, "return_rrtconnect_range", None)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            parser.error("RETURN_RRTCONNECT_RANGE must be finite and nonnegative")
        args.ompl_parameters["rrtconnect"] = {"range": value}
    args.show_config = cli.show_config
    args.output_dir = cli.output_dir
    return args, [planner]


def target_pose(start_pose, position=None, offset=None, rpy_deg=None):
    pose = copy.deepcopy(start_pose)
    if position is not None:
        pose.position.x, pose.position.y, pose.position.z = map(float, position)
    elif offset is not None:
        for axis, distance in zip("xyz", offset):
            setattr(pose.position, axis, getattr(pose.position, axis) + distance)
    if rpy_deg is not None:
        roll, pitch, yaw = [math.radians(v) * 0.5 for v in rpy_deg]
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        pose.orientation.w = cr * cp * cy + sr * sp * sy
    return pose


def cartesian_waypoints(start_pose, args):
    if args.waypoint_positions_m is not None:
        return [target_pose(start_pose, position=p, rpy_deg=args.target_rpy_deg)
                for p in args.waypoint_positions_m]
    return [target_pose(start_pose, offset=p, rpy_deg=args.target_rpy_deg)
            for p in args.waypoint_offsets_m]


def circ_interim(start_pose, target, args):
    if args.arc_position_m is not None:
        interim = target_pose(start_pose, position=args.arc_position_m)
    else:
        midpoint = [(getattr(start_pose.position, a) + getattr(target.position, a)) * 0.5 for a in "xyz"]
        interim = target_pose(start_pose, position=[p + d for p, d in zip(midpoint, args.arc_offset)])
    goal_delta = [getattr(target.position, a) - getattr(start_pose.position, a) for a in "xyz"]
    via_delta = [getattr(interim.position, a) - getattr(start_pose.position, a) for a in "xyz"]
    cross = [goal_delta[1] * via_delta[2] - goal_delta[2] * via_delta[1],
             goal_delta[2] * via_delta[0] - goal_delta[0] * via_delta[2],
             goal_delta[0] * via_delta[1] - goal_delta[1] * via_delta[0]]
    if math.sqrt(sum(v * v for v in cross)) < 1e-9:
        raise RuntimeError("CIRC start / via / target must not be collinear; edit VIA_POSITION_M or VIA_OFFSET_M")
    return interim


def run(planner, parameters, argv=None, source_file=None):
    args, planners = parse_args(planner, parameters, argv)
    args.parameters_file = str(Path(source_file).resolve()) if source_file else "Python PARAMETERS"
    if args.show_config:
        print(json.dumps({"planner": planner, "parameters_file": args.parameters_file,
                          **parameters, "execute": args.execute,
                          "return_to_start": args.return_to_start}, indent=2))
        return 0
    directory = args.output_dir or (Path(__file__).parent / "results" / datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    directory.mkdir(parents=True, exist_ok=True)
    # Keep the ROS context alive during KeyboardInterrupt so the action can be cancelled.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = LeftArmPlanner()
    records = []
    failed = False
    try:
        node.get_logger().info(f"Parameters: {args.parameters_file}; execute={args.execute}, return_to_start={args.return_to_start}")
        node.get_logger().info(f"ROS: {ros_environment_text()}")
        if args.execute:
            node.check_hardware_ready()
        start = node.snapshot()
        start_pose = node.fk(start)
        waypoints = None
        joint_target = None
        if planner == "cartesian":
            waypoints = cartesian_waypoints(start_pose, args)
            target = waypoints[-1]
        elif planner in OMPL_IDS:
            joint_target = [math.radians(v) for v in args.joint_goal_deg]
            target = node.fk(with_positions(start, JOINTS, joint_target))
        else:
            target = target_pose(start_pose, args.target_position_m, args.delta, args.target_rpy_deg)
        delta = [getattr(target.position, a) - getattr(start_pose.position, a) for a in "xyz"]
        if (math.sqrt(sum(v * v for v in delta)) < 1e-6 and joint_target is None
                and waypoints is None and args.target_rpy_deg is None):
            raise RuntimeError("The target displacement is zero")
        interim = circ_interim(start_pose, target, args) if planner == "circ" else None
        config = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
        config.update(group=GROUP, tip=TIP, frame=FRAME, start_joints=state_positions(start),
                      start_xyz=[getattr(start_pose.position, a) for a in "xyz"],
                      target_xyz=[getattr(target.position, a) for a in "xyz"])
        (directory / "request.json").write_text(json.dumps(config, indent=2) + "\n")
        if planner in OMPL_IDS:
            node.get_logger().info(f"{GROUP}, target joints (deg)={args.joint_goal_deg}, execute={args.execute}")
        else:
            node.get_logger().info(f"{GROUP}, {TIP}, target delta={delta}, execute={args.execute}")
        node.get_logger().info(f"Results: {directory.resolve()}")
        for trial in range(1, args.repeat + 1):
            for color_index, planner in enumerate(planners):
                record = {"planner": planner, "trial": trial, "status": "FAILED"}
                records.append(record)
                begin = time.monotonic()
                try:
                    if planner == "cartesian":
                        trajectory, planning_s, fraction = node.plan_cartesian(start, waypoints, args)
                    else:
                        goal = joint_goal(joint_target) if planner in OMPL_IDS else pose_goal(target)
                        trajectory, planning_s, fraction = node.plan_motion(start, goal, planner, args, interim)
                    record.update(planning_s=planning_s, service_wall_s=time.monotonic() - begin, fraction=fraction)
                    record.update(node.preview_and_save(start, trajectory, f"{planner}_{trial:03d}", target, directory, color_index))
                    record["status"] = "PLANNED"
                    node.get_logger().info(
                        f"{planner} #{trial}: plan={planning_s:.4f}s, points={record['points']}, "
                        f"duration={record['duration_s']:.3f}s, EE length~{record['ee_length_mm']:.2f}mm")
                except RuntimeError as exc:
                    failed = True
                    record.update(error=str(exc), service_wall_s=time.monotonic() - begin)
                    node.get_logger().error(f"{planner} #{trial}: {exc}")
                    continue
                until = time.monotonic() + args.pause
                while rclpy.ok() and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.05)
                # Execution / return failures abort the run; never continue from an unknown start.
                if args.execute:
                    try:
                        node.execute(trajectory, start)
                        record["status"] = "EXECUTED"
                        if args.return_to_start:
                            node.get_logger().info("Planning RRTConnect return to the original left-arm joints")
                            actual = node.snapshot()
                            return_goal = joint_goal([state_positions(start)[n] for n in JOINTS])
                            recovery, _, _ = node.plan_motion(actual, return_goal, "rrtconnect", args)
                            node.execute(recovery, actual)
                            record["status"] = "RETURNED"
                    except KeyboardInterrupt:
                        record.update(status="CANCELED", error="Interrupted by Ctrl+C")
                        raise
                    except BaseException as exc:
                        record.update(status="EXECUTION_FAILED", error=str(exc))
                        raise
    except KeyboardInterrupt:
        failed = True
        node.cancel_active()
        node.get_logger().info("Interrupted; active execution cancellation requested")
    except RuntimeError as exc:
        failed = True
        node.get_logger().error(str(exc))
    finally:
        (directory / "summary.json").write_text(json.dumps(records, indent=2) + "\n")
        if records:
            fields = list(dict.fromkeys(k for record in records for k in record))
            with (directory / "summary.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(records)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit("Run ompl_rrtstar.py, ompl_rrtconnect.py, ompl_prm.py, ompl_est.py, pilz_circ.py, cartesian_path.py or pilz_lin.py")
