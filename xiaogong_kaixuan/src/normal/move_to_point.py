#!/usr/bin/env python3

import os

os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, RobotTrajectory


class MoveToZero(Node):
	def __init__(self):
		super().__init__("move_to_zero")
		self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
		self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
		self._last_joint_state = None
		self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

	def _joint_state_cb(self, msg: JointState):
		self._last_joint_state = msg

	def _wait_joint_state(self, timeout_sec=3.0):
		end_time = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
		while rclpy.ok() and self._last_joint_state is None:
			rclpy.spin_once(self, timeout_sec=0.1)
			if self.get_clock().now().nanoseconds > end_time:
				break
		return self._last_joint_state

	def _build_request(self, joint_names, target_positions, pipeline_id):
		req = GetMotionPlan.Request()
		mpr = req.motion_plan_request
		mpr.group_name = "right_arm"
		mpr.pipeline_id = pipeline_id
		mpr.num_planning_attempts = 10
		mpr.allowed_planning_time = 10.0

		# 用当前关节状态作为起点，避免 planner 拿不到起始状态导致失败
		js_msg = self._wait_joint_state(timeout_sec=3.0)
		if js_msg is not None and js_msg.name and js_msg.position:
			name_to_pos = {n: p for n, p in zip(js_msg.name, js_msg.position)}
			start_name, start_pos = [], []
			for n in joint_names:
				if n in name_to_pos:
					start_name.append(n)
					start_pos.append(float(name_to_pos[n]))
			if len(start_name) == len(joint_names):
				mpr.start_state.joint_state.name = start_name
				mpr.start_state.joint_state.position = start_pos

		goal = Constraints()
		for name, pos in zip(joint_names, target_positions):
			jc = JointConstraint()
			jc.joint_name = name
			jc.position = float(pos)
			jc.tolerance_above = 0.02
			jc.tolerance_below = 0.02
			jc.weight = 1.0
			goal.joint_constraints.append(jc)
		mpr.goal_constraints = [goal]

		return req

	def plan_zero(self):
		if not self.plan_cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError("/plan_kinematic_path not available")

		joint_names = [
			"right_joint1",
			"right_joint2",
			"right_joint3",
			"right_joint4",
			"right_joint5",
			"right_joint6",
			"right_joint7",
		]
		target_positions = [
			0.0,  # right_joint1
			0.0,  # right_joint2
			0.0,  # right_joint3
			0.0,  # right_joint4
			0.0,  # right_joint5
			0.0,  # right_joint6
			0.0,  # right_joint7
		]

		# 先尝试显式 OMPL，失败再回退到默认 pipeline
		for pipeline_id in ["ompl", ""]:
			req = self._build_request(joint_names, target_positions, pipeline_id)
			future = self.plan_cli.call_async(req)
			rclpy.spin_until_future_complete(self, future)
			res = future.result()
			if res is None:
				continue

			code = res.motion_plan_response.error_code.val
			self.get_logger().info(
				f"plan pipeline={'default' if pipeline_id == '' else pipeline_id} error_code={code}"
			)
			if code == 1:
				traj = res.motion_plan_response.trajectory
				if not traj.joint_trajectory.points:
					raise RuntimeError("empty trajectory")
				return traj

		# 规划持续失败时，直接构造关节轨迹并通过 /execute_trajectory 下发
		self.get_logger().warn("planning failed, fallback to direct joint trajectory")
		traj = RobotTrajectory()
		traj.joint_trajectory.joint_names = list(joint_names)

		js_msg = self._wait_joint_state(timeout_sec=3.0)
		if js_msg is None or not js_msg.name or not js_msg.position:
			raise RuntimeError("no /joint_states, cannot build fallback trajectory")

		name_to_pos = {n: p for n, p in zip(js_msg.name, js_msg.position)}
		current = []
		for n in joint_names:
			if n not in name_to_pos:
				raise RuntimeError(f"joint {n} not found in /joint_states")
			current.append(float(name_to_pos[n]))

		p0 = JointTrajectoryPoint()
		p0.positions = current
		p0.time_from_start.sec = 0

		p1 = JointTrajectoryPoint()
		p1.positions = [float(x) for x in target_positions]
		p1.time_from_start.sec = 4

		traj.joint_trajectory.points = [p0, p1]
		return traj

	def execute(self, traj):
		if not self.exec_cli.wait_for_server(timeout_sec=5.0):
			raise RuntimeError("/execute_trajectory not available")

		goal = ExecuteTrajectory.Goal()
		goal.trajectory = traj

		send_future = self.exec_cli.send_goal_async(goal)
		rclpy.spin_until_future_complete(self, send_future)
		handle = send_future.result()
		if handle is None or not handle.accepted:
			raise RuntimeError("execute goal rejected")

		result_future = handle.get_result_async()
		rclpy.spin_until_future_complete(self, result_future)
		res = result_future.result().result
		self.get_logger().info(f"execute error_code={res.error_code.val}")


def main():
	rclpy.init()
	node = MoveToZero()
	try:
		traj = node.plan_zero()
		node.execute(traj)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
