#!/usr/bin/env python3
"""Read-only ROS graph / feedback checks; never sends a motion goal."""

import argparse
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]


def model_signature(text):
    robot = ET.fromstring(text)
    signatures = {}
    for joint in robot.findall("joint"):
        elements = []
        for tag in ("parent", "child", "origin", "axis", "limit"):
            element = joint.find(tag)
            attributes = {}
            if element is not None:
                for key, value in element.attrib.items():
                    attributes[key] = tuple(round(float(part), 9) for part in value.split()) if key not in {"link"} else value
            elements.append((tag, sorted(attributes.items())))
        signatures[joint.get("name")] = (joint.get("type"), elements)
    return signatures


class Check(Node):
    def __init__(self, arm):
        super().__init__("kaixuan_remote_connection_check")
        self.arm = arm
        arms = ("left", "right") if arm == "both" else (arm,)
        self.joints = {f"{prefix}_joint{i}" for prefix in arms for i in range(1, 8)}
        self.received = {}
        self.source_delay = None
        self.tf_count = 0
        self.scene_count = 0
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos_profile_sensor_data)
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, static_qos)
        self.create_subscription(PlanningScene, "/monitored_planning_scene", self.on_scene, 5)
        self.plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.description = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def on_joints(self, message):
        for name, value in zip(message.name, message.position):
            if math.isfinite(value):
                self.received[name] = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if stamp:
            self.source_delay = self.get_clock().now().nanoseconds / 1e9 - stamp

    def on_tf(self, message):
        self.tf_count += len(message.transforms)

    def on_scene(self, message):
        self.scene_count += 1

    def rpc(self, client, request):
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=4)
        if not future.done():
            future.cancel()
            return None
        try:
            return future.result()
        except Exception:
            return None

    def run(self, timeout):
        domain = self.context.get_domain_id()
        print(f"ROS Domain={domain}  RMW={os.environ.get('RMW_IMPLEMENTATION', '')}")
        print("只读取状态、查询服务；不会发送规划或运动目标。", flush=True)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.joints <= self.received.keys() and self.tf_count and self.plan.service_is_ready()
                    and self.scene.service_is_ready() and self.move.server_is_ready()
                    and self.execute.server_is_ready() and self.controllers.service_is_ready()
                    and self.description.service_is_ready()):
                break
        problems = []

        if (not self.received and not self.tf_count and not self.plan.service_is_ready()
                and not self.move.server_is_ready() and not self.controllers.service_is_ready()):
            print(f"[提示] Domain {domain} 中尚未发现机器人反馈或服务。请先核对两端 Domain ID 与 DDS 网络。")
            print("环境变量只影响之后启动的进程；在另一个终端 export 不会改变已运行的 bringup。")

        def report(good, title, advice):
            print(f"[{'OK' if good else 'FAIL'}] {title}", flush=True)
            if not good:
                problems.append(advice)

        now = time.monotonic()
        missing = sorted(name for name in self.joints if now - self.received.get(name, 0) > 1)
        report(not missing, "所选机械臂的实时 /joint_states" + (f"，缺少/过时：{missing}" if missing else ""),
               "确认机器人 bringup 已启动，且两端 Domain ID 相同、ROS_LOCALHOST_ONLY=0。")
        report(self.tf_count > 0, "/tf 或 /tf_static", "检查 robot_state_publisher 和 DDS 网络连接。")
        report(self.plan.service_is_ready(), "/plan_kinematic_path", "检查机器人端 move_group 是否正常运行。")
        report(self.move.server_is_ready(), "/move_action", "检查机器人端 MoveIt action 是否可发现。")
        report(self.execute.server_is_ready(), "/execute_trajectory", "检查机器人端轨迹执行 action 是否可发现。")
        response = self.rpc(self.controllers, ListControllers.Request())
        active = {item.name for item in response.controller if item.state == "active"} if response else set()
        required = {f"{prefix}_arm_controller" for prefix in (("left", "right") if self.arm == "both" else (self.arm,))}
        report(required <= active, "已激活的控制器：" + (", ".join(sorted(active)) or "未获取到"),
               "控制器未就绪，或检查的手臂与机器人 active_arm 不一致。")
        scene = self.rpc(self.scene, GetPlanningScene.Request())
        report(scene is not None, "/get_planning_scene 可响应", "检查规划场景服务及双向 DDS 通信。")
        request = GetParameters.Request()
        request.names = ["robot_description"]
        response = self.rpc(self.description, request)
        description = response.values[0].string_value if response and response.values else ""
        if description:
            matching = False
            try:
                matching = model_signature(description) == model_signature((ROOT / "model/robot.urdf").read_text())
                plugin = ET.fromstring(description).findtext("./ros2_control/hardware/plugin")
                if plugin:
                    print("机器人端硬件插件：" + plugin)
            except (ET.ParseError, ValueError):
                pass
            report(matching, "客户端 URDF 关节结构与机器人一致", "已读到机器人模型，但关节结构不一致或无法解析；请更新匹配的客户端模型。")
        else:
            print("[SKIP] URDF 模型比对：未读到机器人模型，暂不能判断是否一致。")
            problems.append("无法读取 robot_state_publisher 的 robot_description；请先检查 DDS 连接及参数服务。")
        if self.source_delay is not None:
            print(f"关节消息时间差（含网络延迟）：{self.source_delay:+.3f} 秒")
            if abs(self.source_delay) > 1:
                problems.append("两台电脑时间相差较大，请检查 NTP / timedatectl，避免 MoveIt 将反馈判断为过期。")
        if problems:
            print("\n需要处理：")
            for item in dict.fromkeys(problems):
                print("- " + item)
            print("网络排查见 README.md；Wi-Fi 禁止组播时可使用包内 Discovery Server 方案。")
            return 1
        print("\n连接检查通过。运行 bash ./start_rviz.sh，在 RViz 中先 Plan 再 Execute。")
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", choices=["left", "right", "both"], default="left")
    parser.add_argument("--timeout", type=float, default=12)
    args = parser.parse_args()
    if not 0 < args.timeout <= 60:
        parser.error("timeout must be between 0 and 60 seconds")
    rclpy.init(args=[])
    node = Check(args.arm)
    try:
        return node.run(args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
