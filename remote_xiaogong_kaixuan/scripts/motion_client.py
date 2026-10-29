#!/usr/bin/env python3
"""Remote joint goals through MoveIt. Planning is the default; execution is explicit."""

import argparse
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from check_connection import model_signature

ROOT = Path(__file__).resolve().parents[1]


def positive(value, label, maximum=None):
    value = float(value)
    if not math.isfinite(value) or value <= 0 or (maximum is not None and value > maximum):
        raise ValueError(f"{label} 必须为大于 0" + (f" 且不超过 {maximum}" if maximum else "") + " 的有限数值")
    return value


def joint_names(arm):
    if arm not in ("left", "right"):
        raise ValueError("arm 必须为 left 或 right")
    return [f"{arm}_joint{i}" for i in range(1, 8)]


def make_goal(arm, angles, *, execute=False, velocity=0.1, acceleration=0.1, planning_time=5.0):
    """Build a MoveGroup goal; angles are ROS joint coordinates in radians."""
    names = joint_names(arm)
    angles = [float(value) for value in angles]
    if len(angles) != 7 or not all(math.isfinite(value) for value in angles):
        raise ValueError("必须提供七个有限关节角度")
    model = ET.parse(ROOT / "model/robot.urdf").getroot()
    for i, (name, value) in enumerate(zip(names, angles)):
        limit = model.find(f"joint[@name='{name}']/limit")
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        if not lower - 1e-6 <= value <= upper + 1e-6:
            raise ValueError(f"{name}={math.degrees(value):.6f}° 超出模型限位 "
                             f"[{math.degrees(lower):.6f}, {math.degrees(upper):.6f}]°")
        angles[i] = max(lower, min(upper, value))  # Ignore only sub-microradian encoder rounding.
    goal = MoveGroup.Goal()
    request = goal.request
    request.group_name = f"{arm}_arm"
    request.pipeline_id = "ompl"
    request.start_state.is_diff = True  # The server supplies its current complete robot state.
    request.num_planning_attempts = 5
    request.allowed_planning_time = positive(planning_time, "planning_time", 60)
    request.max_velocity_scaling_factor = positive(velocity, "velocity", 1)
    request.max_acceleration_scaling_factor = positive(acceleration, "acceleration", 1)
    constraints = Constraints(name="remote_joint_goal")
    for name, value in zip(names, angles):
        constraints.joint_constraints.append(JointConstraint(
            joint_name=name, position=value, tolerance_above=0.001,
            tolerance_below=0.001, weight=1.0))
    request.goal_constraints = [constraints]
    goal.planning_options.plan_only = not execute
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    goal.planning_options.replan = False
    goal.planning_options.look_around = False
    return goal


class ArmClient(Node):
    """Synchronous Python API. Call rclpy.init() first; one outstanding motion per instance."""

    def __init__(self, arm="left"):
        self.joint_names = joint_names(arm)
        super().__init__("kaixuan_remote_motion_client")
        self.arm = arm
        self._positions = {}
        self.create_subscription(JointState, "/joint_states", self._on_state, qos_profile_sensor_data)
        self._description = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        self._controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self._move = ActionClient(self, MoveGroup, "/move_action")
        self._pending_goal = self._handle = self._result = None
        self._cancel_requested = False
        self._motion_pending = False

    def _on_state(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        for name, value in zip(message.name, message.position):
            if math.isfinite(value):
                self._positions[name] = (value, time.monotonic(), stamp)

    def current_joint_angles(self, *, degrees=False, timeout=10.0):
        """Return joint1..7 from fresh feedback; never substitutes missing values with zero."""
        deadline = time.monotonic() + positive(timeout, "timeout")
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if all(name in self._positions and now - self._positions[name][1] <= 1.0
                   for name in self.joint_names):
                ros_now = self.get_clock().now().nanoseconds / 1e9
                if any(stamp and abs(ros_now - stamp) > 1.0
                       for _, _, stamp in (self._positions[name] for name in self.joint_names)):
                    raise RuntimeError("关节反馈时间戳过时或两端时钟不同步，请检查 NTP / timedatectl")
                angles = [self._positions[name][0] for name in self.joint_names]
                return [math.degrees(q) for q in angles] if degrees else angles
        raise RuntimeError(f"未收到 {self.arm} 臂七个关节的实时反馈；请运行 check_connection.sh --arm {self.arm}")

    def _wait(self, future, timeout, label):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f"等待 {label} 超时")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} 未返回结果")
        return result

    def _rpc(self, client, request, label):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"无法发现 {label}；请检查机器人 bringup 和 ROS Domain")
        return self._wait(client.call_async(request), 10.0, label)

    def _check_robot(self):
        response = self._rpc(self._description, GetParameters.Request(names=["robot_description"]), "机器人模型")
        description = response.values[0].string_value if response.values else ""
        if not description or model_signature(description) != model_signature((ROOT / "model/robot.urdf").read_text()):
            raise RuntimeError("机器人与客户端 URDF 不一致；请更新两端并重启 bringup")
        controllers = self._rpc(self._controllers, ListControllers.Request(), "控制器列表")
        if not any(item.name == f"{self.arm}_arm_controller" and item.state == "active"
                   for item in controllers.controller):
            raise RuntimeError(f"{self.arm}_arm_controller 未激活；请核对机器人 active_arm")
        self.current_joint_angles()

    def cancel(self):
        """Return whether cancellation was acknowledged or the action was already complete."""
        if self._cancel_requested or (self._result is not None and self._result.done()):
            return True
        if self._pending_goal is None and self._handle is None:
            return True
        if not self.context.ok():
            print("[FAIL] ROS 已关闭，无法发送取消请求；机械臂可能继续原轨迹，请在现场确认状态。")
            return False
        try:
            if self._handle is None and self._pending_goal is not None:
                self._handle = self._wait(self._pending_goal, 3.0, "运动目标应答")
            if self._handle is not None and self._handle.accepted:
                response = self._wait(self._handle.cancel_goal_async(), 3.0, "取消请求")
                if not response.goals_canceling:
                    if self._result is not None and self._result.done():
                        return True
                    raise RuntimeError("服务端未确认取消")
                self._cancel_requested = True
                self.get_logger().info("服务端已受理取消请求。")
            return True
        except Exception as exc:
            self.get_logger().error(f"取消未获确认：{exc}；机械臂可能继续原轨迹，请在现场确认状态。")
            return False

    def move_joints(self, angles, *, degrees=False, execute=False, velocity=0.1,
                    acceleration=0.1, planning_time=5.0, timeout=120.0):
        """Plan seven absolute joint targets; execute=True also runs the plan on the robot."""
        if (getattr(self, "_motion_pending", False)
                and not (self._result is not None and self._result.done())):
            raise RuntimeError("上一个目标的状态尚未结束，不能发送下一个目标；请先确认执行或取消结果")
        timeout = positive(timeout, "timeout")
        target = [math.radians(q) for q in angles] if degrees else list(angles)
        goal = make_goal(self.arm, target, execute=execute, velocity=velocity,
                         acceleration=acceleration, planning_time=planning_time)
        self._pending_goal = self._handle = self._result = None
        self._cancel_requested = False
        self._check_robot()
        if not self._move.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("无法发现 /move_action，请检查机器人 move_group")
        self.get_logger().info(f"Domain={self.context.get_domain_id()}  {self.arm}_arm  "
                               + ("规划并执行" if execute else "仅规划"))
        try:
            self._motion_pending = True
            self._pending_goal = self._move.send_goal_async(goal)
            self._handle = self._wait(self._pending_goal, 10.0, "MoveGroup 目标应答")
            if not self._handle.accepted:
                self._motion_pending = False
                raise RuntimeError("机器人拒绝了运动目标")
            self._result = self._handle.get_result_async()
            response = self._wait(self._result, timeout, "MoveGroup 规划/执行")
            self._motion_pending = False
            result = response.result
            code = result.error_code.val
            if response.status != GoalStatus.STATUS_SUCCEEDED or code != MoveItErrorCodes.SUCCESS:
                name = next((key for key in dir(MoveItErrorCodes)
                             if key.isupper() and getattr(MoveItErrorCodes, key) == code), "UNKNOWN")
                raise RuntimeError(f"MoveIt 返回 {name} ({code})，action status={response.status}")
            return result
        except BaseException as exc:
            canceled = self.cancel()
            if not canceled and isinstance(exc, RuntimeError):
                raise RuntimeError(f"{exc}；取消未获确认，机械臂可能继续原轨迹") from exc
            raise

    def jog_joint(self, joint, delta_deg, **options):
        """Change one joint by degrees relative to fresh feedback; other targets hold position."""
        if joint not in range(1, 8) or not math.isfinite(delta_deg):
            raise ValueError("joint 必须为 1～7，delta_deg 必须为有限数值")
        target = self.current_joint_angles()
        target[joint - 1] += math.radians(delta_deg)
        return self.move_joints(target, **options)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", choices=["left", "right"], default="left")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--state", action="store_true", help="只读取七个关节的角度")
    mode.add_argument("--joints-deg", nargs=7, type=float, help="七个关节的绝对目标，单位度")
    mode.add_argument("--joint", type=int, choices=range(1, 8), help="相对移动的关节编号")
    parser.add_argument("--delta-deg", type=float, help="配合 --joint 使用的相对角度")
    parser.add_argument("--execute", action="store_true", help="规划成功后执行，真实机械臂会运动")
    parser.add_argument("--velocity", type=float, default=0.1)
    parser.add_argument("--acceleration", type=float, default=0.1)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=120.0, help="规划/执行结果等待秒数")
    args = parser.parse_args()
    if (args.joint is not None) != (args.delta_deg is not None):
        parser.error("--joint 和 --delta-deg 必须同时提供")
    if args.state and args.execute:
        parser.error("--state 不能配合 --execute")
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = ArmClient(args.arm)
    try:
        if args.state:
            for name, q in zip(node.joint_names, node.current_joint_angles(degrees=True)):
                print(f"{name}: {q:+.6f}°")
            return 0
        options = dict(execute=args.execute, velocity=args.velocity, acceleration=args.acceleration,
                       planning_time=args.planning_time, timeout=args.timeout)
        if args.joint is not None:
            result = node.jog_joint(args.joint, args.delta_deg, **options)
        else:
            result = node.move_joints(args.joints_deg, degrees=True, **options)
        trajectory = result.planned_trajectory.joint_trajectory
        print(f"{'执行成功' if args.execute else '规划成功，未执行'}；轨迹点数={len(trajectory.points)}")
        if trajectory.points:
            last = trajectory.points[-1]
            print(f"轨迹时长={last.time_from_start.sec + last.time_from_start.nanosec / 1e9:.3f} 秒")
        return 0
    except KeyboardInterrupt:
        node.cancel()
        return 130
    except (RuntimeError, ValueError) as exc:
        print(f"[FAIL] {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
