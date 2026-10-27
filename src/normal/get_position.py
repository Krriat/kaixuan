#!/usr/bin/env python3

import argparse
import math
import os
import sys
import shlex
from typing import Optional

def _force_udp_only_local_transport() -> None:
	# Root-launched ROS 2 processes on the same host can make Fast DDS shared
	# memory inaccessible to normal-user tools. Force UDP for local helpers.
	os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")


_force_udp_only_local_transport()

def _ensure_ros_runtime_env() -> None:
	if os.environ.get("_GET_POSITION_ROS_ENV_READY") == "1":
		return
	if os.environ.get("AMENT_PREFIX_PATH") and os.environ.get("LD_LIBRARY_PATH"):
		return

	script_path = os.path.abspath(__file__)
	quoted_args = " ".join(shlex.quote(arg) for arg in sys.argv[1:])
	command = (
		"source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
		"source /home/niic/cxl/ros2_control/install/setup.bash >/dev/null 2>&1 && "
		"export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && "
		f"export _GET_POSITION_ROS_ENV_READY=1 && python3 {shlex.quote(script_path)}"
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK


JOINT_PRINT_ORDER = [
	"right_joint1",
	"right_joint2",
	"right_joint3",
	"right_joint4",
	"right_joint5",
	"right_joint6",
	"right_joint7",
	"left_joint1",
	"left_joint2",
	"left_joint3",
	"left_joint4",
	"left_joint5",
	"left_joint6",
	"left_joint7",
]


class FKClient(Node):
	def __init__(self, *, fk_link: str, base_frame: str, joint_states_topic: str):
		super().__init__("get_position_and_fk")
		self._fk_link = fk_link
		self._base_frame = base_frame
		self._js_topic = joint_states_topic
		self._last_js: Optional[JointState] = None
		joint_state_qos = QoSProfile(
			history=HistoryPolicy.KEEP_LAST,
			depth=1,
			reliability=ReliabilityPolicy.RELIABLE,
			durability=DurabilityPolicy.TRANSIENT_LOCAL,
		)

		self.create_subscription(JointState, self._js_topic, self._on_js, joint_state_qos)
		self._fk = self.create_client(GetPositionFK, "/compute_fk")

	def _on_js(self, msg: JointState) -> None:
		# Keep the latest; we only need one snapshot.
		self._last_js = msg

	def wait_for_joint_state(self, timeout_s: float) -> JointState:
		end = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)
		while rclpy.ok() and self._last_js is None and self.get_clock().now() < end:
			rclpy.spin_once(self, timeout_sec=0.1)
		if self._last_js is None:
			raise TimeoutError(f"No messages received on {self._js_topic}")
		return self._last_js

	def call_fk(self, js: JointState, timeout_s: float):
		if not self._fk.wait_for_service(timeout_sec=timeout_s):
			raise TimeoutError("/compute_fk service not available")

		req = GetPositionFK.Request()
		req.header.frame_id = self._base_frame
		req.fk_link_names = [self._fk_link]
		req.robot_state.joint_state = js

		future = self._fk.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
		if not future.done() or future.result() is None:
			raise TimeoutError("/compute_fk call timed out")
		return future.result()


def main():
	ap = argparse.ArgumentParser(description="Read /joint_states and call MoveIt /compute_fk")
	ap.add_argument("--fk-link", default="right_link7", help="Link name to compute FK for")
	ap.add_argument("--base-frame", default="base_link", help="Frame for returned FK pose")
	ap.add_argument("--joint-states-topic", default="/joint_states")
	ap.add_argument("--timeout", type=float, default=3.0)
	args = ap.parse_args()

	rclpy.init()
	node = FKClient(
		fk_link=args.fk_link,
		base_frame=args.base_frame,
		joint_states_topic=args.joint_states_topic,
	)

	try:
		js = node.wait_for_joint_state(timeout_s=args.timeout)
		node.get_logger().info(f"Got JointState with {len(js.name)} joints")
		joint_map = {name: position for name, position in zip(js.name, js.position)}

		print("\n== Joint angles (deg) ==")
		for joint_name in JOINT_PRINT_ORDER:
			if joint_name in joint_map:
				print(f"{joint_name}: {math.degrees(joint_map[joint_name]):.6f}")

		remaining_joints = [name for name in js.name if name not in JOINT_PRINT_ORDER]
		for joint_name in remaining_joints:
			print(f"{joint_name}: {math.degrees(joint_map[joint_name]):.6f}")

		res = node.call_fk(js, timeout_s=args.timeout)
		print("\n== FK result ==")
		print(f"error_code: {res.error_code.val}")
		if res.pose_stamped:
			ps = res.pose_stamped[0]
			p = ps.pose.position
			q = ps.pose.orientation
			print(f"frame_id: {ps.header.frame_id}")
			print(f"position:  x={p.x:.6f} y={p.y:.6f} z={p.z:.6f}")
			print(f"orientation: x={q.x:.6f} y={q.y:.6f} z={q.z:.6f} w={q.w:.6f}")
		else:
			print("pose_stamped: []")
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
