#!/usr/bin/env python3

import os

os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK
from moveit_msgs.action import ExecuteTrajectory


class P2P(Node):
	def __init__(self):
		super().__init__("point_to_point_cartesian")

		self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
		self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
		self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
		self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

	def params(self):
		# ---- 可改参数都放这里 ----
		group_name = "right_arm"
		ee_link = "right_link7"
		base_frame = "base_link"

		joint_names = [
			"right_joint1",
			"right_joint2",
			"right_joint3",
			"right_joint4",
			"right_joint5",
			"right_joint6",
			"right_joint7",
		]

		# IK seed（用于pose_a求逆解）
		q_a = [1.313011, -0.000002, 0.000095, -1.320018, -0.000081, -0.000006, 0.000095]

		# A pose
		pose_a = Pose()
		pose_a.position.x, pose_a.position.y, pose_a.position.z = 0.328, -0.526, 0.377
		pose_a.orientation.x, pose_a.orientation.y, pose_a.orientation.z, pose_a.orientation.w = (
			0.999998,
			0.001462,
			0.000001,
			-0.001176,
		)

		# B pose
		pose_b = Pose()
		pose_b.position.x, pose_b.position.y, pose_b.position.z = 0.328, -0.026, 0.377
		pose_b.orientation.x, pose_b.orientation.y, pose_b.orientation.z, pose_b.orientation.w = (
			0.999998,
			0.001462,
			0.000001,
			-0.001176,
		)

		move_to_a_time = 3.0
		max_step = 0.005

		return group_name, ee_link, base_frame, joint_names, q_a, pose_a, pose_b, move_to_a_time, max_step

	def solve_ik_for_pose(self, group_name, ee_link, base_frame, joint_names, q_seed, target_pose):
		if not self.ik_cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError("/compute_ik service not available")

		req = GetPositionIK.Request()
		ik = req.ik_request
		ik.group_name = group_name
		ik.ik_link_name = ee_link
		ik.avoid_collisions = False
		ik.pose_stamped.header.frame_id = base_frame
		ik.pose_stamped.pose = target_pose

		ik.robot_state.joint_state.name = list(joint_names)
		ik.robot_state.joint_state.position = [float(x) for x in q_seed]
		ik.timeout.sec = 1
		ik.timeout.nanosec = 0

		future = self.ik_cli.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		res = future.result()
		if res is None:
			raise RuntimeError("compute_ik failed")

		code = res.error_code.val
		self.get_logger().info(f"IK for A error_code: {code}")
		if code != 1:
			raise RuntimeError(f"IK for pose A failed (error_code={code})")

		name_to_pos = {n: p for n, p in zip(res.solution.joint_state.name, res.solution.joint_state.position)}
		q_sol = []
		for n in joint_names:
			if n not in name_to_pos:
				raise RuntimeError(f"IK solution missing joint: {n}")
			q_sol.append(float(name_to_pos[n]))

		return q_sol

	def plan_to_a_ompl(self, group_name, joint_names, q_a, move_time):
		if not self.plan_cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError("/plan_kinematic_path service not available")

		req = GetMotionPlan.Request()
		mpr = req.motion_plan_request
		mpr.group_name = group_name
		mpr.pipeline_id = "ompl"
		mpr.num_planning_attempts = 5
		mpr.allowed_planning_time = max(2.0, float(move_time))
		mpr.max_velocity_scaling_factor = 0.2
		mpr.max_acceleration_scaling_factor = 0.2

		goal_constraints = Constraints()
		goal_constraints.name = "goal_A"
		for name, pos in zip(joint_names, q_a):
			jc = JointConstraint()
			jc.joint_name = name
			jc.position = float(pos)
			jc.tolerance_above = 1e-3
			jc.tolerance_below = 1e-3
			jc.weight = 1.0
			goal_constraints.joint_constraints.append(jc)
		mpr.goal_constraints = [goal_constraints]

		future = self.plan_cli.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		res = future.result()
		if res is None:
			raise RuntimeError("plan_kinematic_path failed")

		code = res.motion_plan_response.error_code.val
		self.get_logger().info(f"OMPL plan-to-A error_code: {code}")
		if code != 1:
			raise RuntimeError(f"OMPL planning to A failed (error_code={code})")

		traj = res.motion_plan_response.trajectory

		# 让后续笛卡尔段从“规划到达后的真实终点”开始，避免关节顺序/数值偏差
		q_start = list(q_a)
		jt = traj.joint_trajectory
		if jt.joint_names and jt.points:
			last = jt.points[-1].positions
			idx = {n: i for i, n in enumerate(jt.joint_names)}
			for i, name in enumerate(joint_names):
				j = idx.get(name)
				if j is not None and j < len(last):
					q_start[i] = float(last[j])

		return traj, q_start

	def compute_cartesian(self, group_name, ee_link, base_frame, joint_names, q_a, pose_b, max_step):
		if not self.cart_cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError("/compute_cartesian_path service not available")

		req = GetCartesianPath.Request()
		req.header.frame_id = base_frame
		req.group_name = group_name
		req.link_name = ee_link

		# 关键：从A点关节状态作为笛卡尔路径起点
		js = JointState()
		js.name = list(joint_names)
		js.position = list(q_a)
		req.start_state.joint_state = js

		# 直线段：A -> B，只需要把B当作waypoint
		req.waypoints = [pose_b]
		req.max_step = max_step
		req.jump_threshold = 0.0
		req.prismatic_jump_threshold = 0.0
		req.revolute_jump_threshold = 0.0
		req.avoid_collisions = False

		future = self.cart_cli.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		res = future.result()
		if res is None:
			raise RuntimeError("compute_cartesian_path failed")

		self.get_logger().info(f"Cartesian fraction: {res.fraction:.3f}, error_code: {res.error_code.val}")
		if res.fraction < 0.999:
			raise RuntimeError("Cartesian path not fully solved (fraction < 1.0)")

		return res.solution

	def execute_traj(self, traj):
		if not self.exec_cli.wait_for_server(timeout_sec=5.0):
			raise RuntimeError("/execute_trajectory action server not available")

		goal = ExecuteTrajectory.Goal()
		goal.trajectory = traj

		gh_future = self.exec_cli.send_goal_async(goal)
		rclpy.spin_until_future_complete(self, gh_future)
		gh = gh_future.result()
		if gh is None or not gh.accepted:
			raise RuntimeError("ExecuteTrajectory goal rejected")

		res_future = gh.get_result_async()
		rclpy.spin_until_future_complete(self, res_future)
		res = res_future.result().result
		self.get_logger().info(f"ExecuteTrajectory result error_code: {res.error_code.val}")

	def run(self):
		(
			group_name,
			ee_link,
			base_frame,
			joint_names,
			q_seed,
			pose_a,
			pose_b,
			move_to_a_time,
			max_step,
		) = self.params()

		self.get_logger().info("0) Solve IK for pose A (q_seed only as reference)")
		q_a = self.solve_ik_for_pose(group_name, ee_link, base_frame, joint_names, q_seed, pose_a)

		self.get_logger().info("1) Plan to A with OMPL")
		traj_to_a, q_start = self.plan_to_a_ompl(group_name, joint_names, q_a, move_to_a_time)

		self.get_logger().info("2) Execute OMPL trajectory to A")
		self.execute_traj(traj_to_a)

		self.get_logger().info("3) Compute Cartesian path A->B (straight line)")
		traj = self.compute_cartesian(group_name, ee_link, base_frame, joint_names, q_start, pose_b, max_step)

		self.get_logger().info("4) Execute trajectory (check RViz)")
		self.execute_traj(traj)


def main():
	rclpy.init()
	node = P2P()
	try:
		node.run()
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
