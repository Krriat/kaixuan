#!/usr/bin/env python3

import os
import time
from typing import List, Tuple

os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState


# ---- Seed / initial guess (rad) ----
JOINT_NAMES: List[str] = [
	"right_joint1",
	"right_joint2",
	"right_joint3",
	"right_joint4",
	"right_joint5",
	"right_joint6",
	"right_joint7",
]

SEED_Q: List[float] = [
	1.257585,
	0.494358,
	-0.173054,
	-1.081383,
	0.172672,
	-0.259550,
	0.352582,
]


def target_pose() -> Tuple[str, str, Tuple[float, float, float], Tuple[float, float, float, float], float]:
	"""Edit parameters here.

	Returns:
	  group_name, ik_link_name, (x,y,z), (qx,qy,qz,qw), timeout_sec
	"""

	group_name = "right_arm"
	ik_link_name = "right_link7"  # change if your end-effector link differs

	# ---- target pose in base_link ----
	x, y, z = 0.328, -0.526, 0.377
	qx, qy, qz, qw = 0.999998, 0.001462, 0.000001, -0.001176

	# IK solver timeout (inside MoveIt). If no solution in this time, MoveIt returns NO_IK_SOLUTION/TIMED_OUT.
	timeout_sec = 0.05
	return group_name, ik_link_name, (x, y, z), (qx, qy, qz, qw), timeout_sec


class IKClient(Node):
	def __init__(self):
		super().__init__("compute_ik_client")
		self._cli = self.create_client(GetPositionIK, "/compute_ik")

	def compute_ik(self) -> None:
		group_name, ik_link_name, (x, y, z), (qx, qy, qz, qw), timeout_sec = target_pose()

		if not self._cli.wait_for_service(timeout_sec=3.0):
			raise RuntimeError("/compute_ik service not available")

		req = GetPositionIK.Request()
		req.ik_request.group_name = group_name
		req.ik_request.ik_link_name = ik_link_name
		req.ik_request.avoid_collisions = False

		# Seed robot state
		js = JointState()
		js.name = list(JOINT_NAMES)
		js.position = list(SEED_Q)
		req.ik_request.robot_state.joint_state = js

		# Target pose
		ps = PoseStamped()
		ps.header.frame_id = "base_link"
		ps.pose.position.x = float(x)
		ps.pose.position.y = float(y)
		ps.pose.position.z = float(z)
		ps.pose.orientation.x = float(qx)
		ps.pose.orientation.y = float(qy)
		ps.pose.orientation.z = float(qz)
		ps.pose.orientation.w = float(qw)
		req.ik_request.pose_stamped = ps

		# Timeout
		req.ik_request.timeout.sec = int(timeout_sec)
		req.ik_request.timeout.nanosec = int((timeout_sec - int(timeout_sec)) * 1e9)

		# How long we wait for the service *response* to come back.
		# If your ROS graph has multiple move_group instances, calls may hang.
		max_wait_sec = 15.0

		t0 = time.perf_counter()
		future = self._cli.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=max_wait_sec)
		dt_ms = (time.perf_counter() - t0) * 1000.0

		if not future.done() or future.result() is None:
			raise RuntimeError(
				f"/compute_ik call failed (no response). elapsed={dt_ms:.2f} ms\n"
				"Hint: check you have exactly one move_group running (duplicate nodes can cause service calls to hang)."
			)

		res = future.result()

		print(f"compute_ik elapsed: {dt_ms:.2f} ms")
		print(f"error_code: {res.error_code.val}")

		sol = res.solution.joint_state
		if not sol.name:
			print("No solution joints returned")
			return

		print("\n== IK solution (rad) ==")
		for n, p in zip(sol.name, sol.position):
			print(f"{n}: {p:.6f}")


def main() -> None:
	rclpy.init()
	node = IKClient()
	try:
		node.compute_ik()
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
