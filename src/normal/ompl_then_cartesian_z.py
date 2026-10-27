#!/usr/bin/env python3

import argparse
import math
import os

os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [f"right_joint{i}" for i in range(1, 8)]
TARGET_DEGREES = [30.0, 0.0, 0.0, 30.0, 0.0, 30.0, 0.0]
TARGET_RADIANS = [math.radians(value) for value in TARGET_DEGREES]
GROUP_NAME = "right_arm"
BASE_FRAME = "base_link"
TIP_LINK = "right_link7"


def seconds_to_duration(seconds: float) -> Duration:
    nanoseconds = max(0, round(seconds * 1_000_000_000))
    return Duration(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )


class OmplThenCartesianZ(Node):
    def __init__(self) -> None:
        super().__init__("ompl_then_cartesian_z")
        self._joint_state = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self._plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._fk = self.create_client(GetPositionFK, "/compute_fk")
        self._execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_state = message

    def wait_for_joint_state(self, timeout: float = 5.0) -> JointState:
        self._joint_state = None
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_state is not None:
                return self._joint_state
        raise RuntimeError("No fresh /joint_states message received")

    @staticmethod
    def robot_state(joint_state: JointState) -> RobotState:
        state = RobotState()
        state.joint_state = joint_state
        state.is_diff = False
        return state

    @staticmethod
    def right_joint_positions(joint_state: JointState) -> list[float]:
        positions = dict(zip(joint_state.name, joint_state.position))
        missing = [name for name in JOINT_NAMES if name not in positions]
        if missing:
            raise RuntimeError(f"Missing joints in /joint_states: {missing}")
        return [float(positions[name]) for name in JOINT_NAMES]

    def wait_for_moveit(self) -> None:
        services = [
            (self._plan, "/plan_kinematic_path"),
            (self._cartesian, "/compute_cartesian_path"),
            (self._fk, "/compute_fk"),
        ]
        for client, name in services:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"MoveIt service is unavailable: {name}")
        if not self._execute.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveIt action is unavailable: /execute_trajectory")

    def plan_joint_target(self, joint_state: JointState, velocity_scale: float):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = GROUP_NAME
        motion.pipeline_id = "ompl"
        motion.num_planning_attempts = 10
        motion.allowed_planning_time = 10.0
        motion.max_velocity_scaling_factor = velocity_scale
        motion.max_acceleration_scaling_factor = velocity_scale
        motion.start_state = self.robot_state(joint_state)

        goal = Constraints()
        for name, target in zip(JOINT_NAMES, TARGET_RADIANS):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = target
            constraint.tolerance_above = math.radians(0.5)
            constraint.tolerance_below = math.radians(0.5)
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)
        motion.goal_constraints = [goal]

        future = self._plan.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        response = future.result()
        if response is None:
            raise RuntimeError("OMPL planning service returned no response")
        result = response.motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"OMPL planning failed, error_code={result.error_code.val}")
        if not result.trajectory.joint_trajectory.points:
            raise RuntimeError("OMPL returned an empty trajectory")
        self.get_logger().info(
            f"OMPL plan ready: {len(result.trajectory.joint_trajectory.points)} points, "
            f"planning_time={result.planning_time:.3f}s"
        )
        return result.trajectory

    def execute(self, trajectory, label: str) -> None:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._execute.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label} execution goal was rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"{label} execution timed out")
        code = wrapped.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{label} execution failed, error_code={code}")
        self.get_logger().info(f"{label} execution succeeded")

    def current_tip_pose(self, joint_state: JointState):
        request = GetPositionFK.Request()
        request.header.frame_id = BASE_FRAME
        request.fk_link_names = [TIP_LINK]
        request.robot_state = self.robot_state(joint_state)
        future = self._fk.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            code = response.error_code.val if response is not None else "no response"
            raise RuntimeError(f"FK failed, error_code={code}")
        if not response.pose_stamped:
            raise RuntimeError("FK returned no end-effector pose")
        return response.pose_stamped[0].pose

    def plan_cartesian_z(
        self,
        joint_state: JointState,
        distance: float,
        max_step: float,
        duration: float,
    ):
        start_pose = self.current_tip_pose(joint_state)
        target_pose = type(start_pose)()
        target_pose.position.x = start_pose.position.x
        target_pose.position.y = start_pose.position.y
        target_pose.position.z = start_pose.position.z + distance
        target_pose.orientation = start_pose.orientation

        request = GetCartesianPath.Request()
        request.header.frame_id = BASE_FRAME
        request.start_state = self.robot_state(joint_state)
        request.group_name = GROUP_NAME
        request.link_name = TIP_LINK
        request.waypoints = [target_pose]
        request.max_step = max_step
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = math.radians(5.0)
        request.avoid_collisions = True

        self.get_logger().info(
            f"Cartesian request: {TIP_LINK} z={start_pose.position.z:.6f} -> "
            f"{target_pose.position.z:.6f} m, max_step={max_step:.4f} m"
        )
        future = self._cartesian.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        response = future.result()
        if response is None:
            raise RuntimeError("Cartesian planning service returned no response")
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"Cartesian planning failed, error_code={response.error_code.val}"
            )
        if response.fraction < 0.999:
            raise RuntimeError(
                f"Cartesian path incomplete: fraction={response.fraction:.4f}"
            )

        trajectory = response.solution
        points = trajectory.joint_trajectory.points
        if len(points) < 2:
            raise RuntimeError("Cartesian path contains fewer than two trajectory points")

        # GetCartesianPath preserves the Cartesian geometry but does not guarantee
        # executable timestamps. Apply uniform timing to the 1 mm waypoints.
        last_index = len(points) - 1
        for index, point in enumerate(points):
            point.time_from_start = seconds_to_duration(duration * index / last_index)
            point.velocities = []
            point.accelerations = []
        trajectory.joint_trajectory.header.stamp.sec = 0
        trajectory.joint_trajectory.header.stamp.nanosec = 0

        self.get_logger().info(
            f"Cartesian path ready: fraction={response.fraction:.4f}, "
            f"points={len(points)}, duration={duration:.3f}s"
        )
        return trajectory


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OMPL to [30,0,0,30,0,30,0] deg, then Cartesian +Z 15 mm"
    )
    parser.add_argument("--z-distance", type=float, default=0.015)
    parser.add_argument("--cartesian-duration", type=float, default=2.0)
    parser.add_argument("--max-step", type=float, default=0.001)
    parser.add_argument("--velocity-scale", type=float, default=0.1)
    parser.add_argument(
        "--cycles",
        type=int,
        default=5,
        help="Number of complete OMPL + Cartesian cycles; use 0 to run until Ctrl+C",
    )
    args = parser.parse_args()
    if not 0.0 < args.z_distance <= 0.05:
        parser.error("--z-distance must be in (0, 0.05] meters")
    if args.cartesian_duration <= 0.0:
        parser.error("--cartesian-duration must be positive")
    if not 0.0 < args.max_step <= 0.005:
        parser.error("--max-step must be in (0, 0.005] meters")
    if not 0.0 < args.velocity_scale <= 1.0:
        parser.error("--velocity-scale must be in (0, 1]")
    if args.cycles < 0:
        parser.error("--cycles must be zero or positive")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = OmplThenCartesianZ()
    try:
        node.wait_for_moveit()
        cycle = 0
        node.get_logger().info(
            f"Target joint pose (deg): {TARGET_DEGREES}; "
            f"cycles={args.cycles or 'infinite'}"
        )
        while rclpy.ok() and (args.cycles == 0 or cycle < args.cycles):
            cycle += 1
            cycle_label = f"{cycle}/{args.cycles}" if args.cycles else str(cycle)
            node.get_logger().info(f"===== Cycle {cycle_label}: OMPL joint move =====")

            initial_state = node.wait_for_joint_state()
            ompl_trajectory = node.plan_joint_target(initial_state, args.velocity_scale)
            node.execute(ompl_trajectory, f"Cycle {cycle_label} OMPL joint move")

            reached_state = node.wait_for_joint_state()
            reached = node.right_joint_positions(reached_state)
            maximum_error = max(
                abs(actual - target) for actual, target in zip(reached, TARGET_RADIANS)
            )
            node.get_logger().info(
                f"Cycle {cycle_label} joint target maximum error: "
                f"{math.degrees(maximum_error):.4f} deg"
            )
            if maximum_error > math.radians(2.0):
                raise RuntimeError("Joint target was not reached within 2 degrees")

            node.get_logger().info(f"===== Cycle {cycle_label}: Cartesian +Z move =====")
            cartesian_trajectory = node.plan_cartesian_z(
                reached_state,
                args.z_distance,
                args.max_step,
                args.cartesian_duration,
            )
            node.execute(cartesian_trajectory, f"Cycle {cycle_label} Cartesian +Z move")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except Exception as exception:
        node.get_logger().error(str(exception))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
