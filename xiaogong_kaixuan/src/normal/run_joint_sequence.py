#!/usr/bin/env python3
"""
运行指定关节的固定动作序列：
  1. 移动到 90°（以 50% 速度）
  2. 等待 3 秒
  3. 移动回 0°（以 20% 速度）

修改下方"用户配置"区域来选择手臂、关节和最大速度。
"""

import math
import os
import shlex
import sys
import time
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
    command = (
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
        "source /home/niic/cxl/ros2_control/install/setup.bash >/dev/null 2>&1 && "
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && "
        f"export _JOG_JOINT_ROS_ENV_READY=1 && python3 {shlex.quote(script_path)}"
    )
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

# ── 配置统一由 joint_config.py 管理，修改那里即可 ───────────────────────────
from joint_config import (  # noqa: E402
    ARM, JOINT_IDX,
    MAX_SPEED_DEG_PER_SEC, FROM_DEG, TO_DEG,
    SPEED_TO, SPEED_BACK, SLEEP_S,
    JOINT_NAME, CONTROLLER_NAME,
)

TIMEOUT_S = 10.0             # 服务 / 动作超时（秒）
GOAL_TIME_TOL_S = 5.0        # goal_time_tolerance（秒）
JOINT_STATES_TOPIC = "/joint_states"
# ─────────────────────────────────────────────────────────────────────────────

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
        "right_joint1", "right_joint2", "right_joint3", "right_joint4",
        "right_joint5", "right_joint6", "right_joint7",
    ],
    "left": [
        "left_joint1", "left_joint2", "left_joint3", "left_joint4",
        "left_joint5", "left_joint6", "left_joint7",
    ],
}

RESULT_CODE_TEXT = {
    0: "SUCCESSFUL", -1: "INVALID_GOAL", -2: "INVALID_JOINTS",
    -3: "OLD_HEADER_TIMESTAMP", -4: "PATH_TOLERANCE_VIOLATED",
    -5: "GOAL_TOLERANCE_VIOLATED",
}


def _seconds_to_duration(seconds: float) -> Duration:
    total_ns = max(0, int(round(seconds * 1e9)))
    return Duration(sec=total_ns // 1_000_000_000, nanosec=total_ns % 1_000_000_000)


def _duration_from_speed(delta_deg: float, speed_fraction: float) -> float:
    """根据角度差和速度百分比计算运动时长（秒）。"""
    if speed_fraction <= 0.0:
        raise ValueError("speed_fraction 必须 > 0")
    effective_speed = MAX_SPEED_DEG_PER_SEC * speed_fraction
    return abs(delta_deg) / effective_speed


class JointMover(Node):
    def __init__(self) -> None:
        super().__init__("joint_mover")
        self._last_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10)
        self._list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self._switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg

    def wait_for_joint_state(self, timeout_s: float) -> JointState:
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)
        while rclpy.ok() and self._last_joint_state is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._last_joint_state is None:
            raise TimeoutError("未收到 /joint_states 消息")
        return self._last_joint_state

    def _list_controllers(self, timeout_s: float):
        if not self._list_client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError("/controller_manager/list_controllers 不可用")
        future = self._list_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        response = future.result()
        if response is None:
            raise RuntimeError("list_controllers 调用失败")
        return response.controller

    def ensure_controller_active(self, controller_name: str) -> None:
        controllers = self._list_controllers(TIMEOUT_S)
        state = next((c.state for c in controllers if c.name == controller_name), None)
        if state is None:
            raise RuntimeError(f"控制器未找到：{controller_name}")
        if state == "active":
            return

        self.get_logger().info(f"激活控制器：{controller_name}")
        if not self._switch_client.wait_for_service(timeout_sec=TIMEOUT_S):
            raise TimeoutError("/controller_manager/switch_controller 不可用")

        req = SwitchController.Request()
        req.activate_controllers = [controller_name]
        req.deactivate_controllers = []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=max(1, int(TIMEOUT_S)), nanosec=0)
        future = self._switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT_S)
        response = future.result()
        if response is None or not response.ok:
            raise RuntimeError(f"激活控制器失败：{controller_name}")

        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=TIMEOUT_S)
        while rclpy.ok() and self.get_clock().now() < deadline:
            controllers = self._list_controllers(1.0)
            state = next((c.state for c in controllers if c.name == controller_name), None)
            if state == "active":
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(f"控制器 {controller_name} 未能在规定时间内变为 active")

    def move_joint(
        self,
        controller_name: str,
        joint_names: list[str],
        current_positions: list[float],
        target_positions: list[float],
        duration_s: float,
    ) -> None:
        action_name = f"/{controller_name}/follow_joint_trajectory"
        client = ActionClient(self, FollowJointTrajectory, action_name)
        if not client.wait_for_server(timeout_sec=TIMEOUT_S):
            raise TimeoutError(f"动作服务器不可用：{action_name}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        p0 = JointTrajectoryPoint()
        p0.positions = list(current_positions)
        p0.time_from_start = Duration(sec=0, nanosec=0)

        p1 = JointTrajectoryPoint()
        p1.positions = list(target_positions)
        p1.time_from_start = _seconds_to_duration(duration_s)

        goal.trajectory.points = [p0, p1]
        goal.goal_time_tolerance = _seconds_to_duration(GOAL_TIME_TOL_S)

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=TIMEOUT_S)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("轨迹目标被拒绝")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_s + TIMEOUT_S)
        wrapped = result_future.result()
        if wrapped is None:
            raise TimeoutError("未收到 FollowJointTrajectory 结果")

        result = wrapped.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            code_name = RESULT_CODE_TEXT.get(result.error_code, f"UNKNOWN({result.error_code})")
            raise RuntimeError(f"轨迹执行失败：{code_name} - {result.error_string}")


def main() -> None:
    if ARM not in JOINTS_BY_ARM:
        raise SystemExit(f"ARM 配置错误：{ARM}，必须是 'left' 或 'right'")
    if not (0 <= JOINT_IDX <= 6):
        raise SystemExit(f"JOINT_IDX 配置错误：{JOINT_IDX}，必须是 0~6")

    joint_names = JOINTS_BY_ARM[ARM]
    target_joint = joint_names[JOINT_IDX]
    controller_name = f"{ARM}_arm_controller"

    rclpy.init()
    node = JointMover()

    try:
        node.ensure_controller_active(controller_name)

        # ── 第一步：读取当前关节状态 ──────────────────────────────────────────
        js = node.wait_for_joint_state(timeout_s=TIMEOUT_S)
        current_map = {name: pos for name, pos in zip(js.name, js.position)}
        missing = [n for n in joint_names if n not in current_map]
        if missing:
            raise RuntimeError(f"joint_states 中缺少关节：{missing}")

        # ── 第二步：移动到 TO_DEG（50% 速度）────────────────────────────────
        current_positions = [current_map[n] for n in joint_names]
        target_positions  = list(current_positions)
        idx = JOINT_IDX
        target_positions[idx] = math.radians(TO_DEG)
        start_deg = math.degrees(current_positions[idx])

        dur_to = _duration_from_speed(TO_DEG - start_deg, SPEED_TO)
        node.get_logger().info(
            f"[1/3] {target_joint}: {start_deg:.1f}° -> {TO_DEG:.1f}°  "
            f"速度={SPEED_TO*100:.0f}%  时长={dur_to:.2f}s"
        )
        node.move_joint(controller_name, joint_names, current_positions, target_positions, dur_to)

        # ── 第三步：等待 SLEEP_S 秒 ──────────────────────────────────────────
        node.get_logger().info(f"[2/3] 已到达 {TO_DEG:.1f}°，等待 {SLEEP_S:.1f} 秒…")
        time.sleep(SLEEP_S)

        # ── 第四步：移动回 FROM_DEG（20% 速度）──────────────────────────────
        after_positions = list(target_positions)          # 以目标位置作为新起点
        back_positions  = list(after_positions)
        back_positions[idx] = math.radians(FROM_DEG)

        dur_back = _duration_from_speed(TO_DEG - FROM_DEG, SPEED_BACK)
        node.get_logger().info(
            f"[3/3] {target_joint}: {TO_DEG:.1f}° -> {FROM_DEG:.1f}°  "
            f"速度={SPEED_BACK*100:.0f}%  时长={dur_back:.2f}s"
        )
        node.move_joint(controller_name, joint_names, after_positions, back_positions, dur_back)

        # ── 结束：读取实际角度 ───────────────────────────────────────────────
        final_js = node.wait_for_joint_state(timeout_s=TIMEOUT_S)
        final_map = {name: pos for name, pos in zip(final_js.name, final_js.position)}
        actual_deg = math.degrees(final_map.get(target_joint, math.nan))
        node.get_logger().info(f"完成。{target_joint} 实际角度 = {actual_deg:.3f}°")

    except Exception as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
