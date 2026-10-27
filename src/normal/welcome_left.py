#!/usr/bin/env python3

import math
import threading
import sys
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK

try:
	from commonly_function import CommonlyFunction
except ModuleNotFoundError:
	sys.path.append(str(Path(__file__).resolve().parents[1]))
	from commonly_function import CommonlyFunction

from commonly_function_left import CommonlyFunctionLeft

class WelcomeController(Node, CommonlyFunctionLeft):
	def __init__(self):
		super().__init__("welcome_controller")

		self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
		self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
		self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
		self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

		self.OMPL_MAX_VELOCITY_SCALING = 0.15
		self.OMPL_MAX_ACCELERATION_SCALING = 0.10
		self.step_goal_tolerance_rad = math.radians(5.0)

		self.group_name = "left_arm"
		self.q_a = [x * -1 for x in [1.256, -0.105, 0.506, 1.868, 0.0, 0.0, -1.082]]
		self.q_b = [x * -1 for x in [1.256, -0.105, -0.506, 1.868, 0.0, 0.0, -1.082]]
		self.q_2 = [x * -1 for x in [1.0, -0.140510, -0.620728, 0.605678, -1.576051, -0.182540, 2.020647]]
		self.q_3 = [angle_deg / 180.0 * math.pi for angle_deg in [0,0,0,0,0,0,0]]

		self.get_logger().info("Started. Execute welcome motion directly.")

	def _move_ompl(self, q_goal, planning_time=4.0, execution_time=4.0):
		self.execute_ompl_goal(
			q_goal=q_goal,
			planning_time=planning_time,
			execution_time=execution_time,
			group_name=self.group_name,
		)
		self._verify_step_reached(q_goal)

	def _verify_step_reached(self, q_goal):
		actual_q = self.get_current_joint_positions(self.group_name, timeout_sec=1.5)
		if actual_q is None:
			raise RuntimeError("Cannot verify welcome step: no current joint positions received")

		joint_names = self.get_group_joint_names(self.group_name)
		mismatches = []
		for joint_name, actual, target in zip(joint_names, actual_q, q_goal):
			error = abs(float(actual) - float(target))
			if error > self.step_goal_tolerance_rad:
				mismatches.append(
					f"{joint_name}: actual={math.degrees(actual):.2f} deg, "
					f"target={math.degrees(target):.2f} deg, error={math.degrees(error):.2f} deg"
				)

		if mismatches:
			raise RuntimeError(
				"Welcome step finished but joints did not reach target within 5 deg: "
				+ "; ".join(mismatches)
			)

	def run_welcome_sequence(self):
		sequence = [
			("q_a", self.q_a, 4.0, 4.0),
			("q_b", self.q_b, 4.0, 4.0),
			("q_a", self.q_a, 4.0, 4.0),
			("q_2", self.q_2, 5.0, 5.0),
			("q_3", self.q_3, 6.0, 6.0),
		]
		try:
			self.get_logger().info("Execute A -> B -> A -> target")
			self.get_logger().info(
				f"Using conservative OMPL scaling: velocity={self.OMPL_MAX_VELOCITY_SCALING:.2f}, "
				f"acceleration={self.OMPL_MAX_ACCELERATION_SCALING:.2f}"
			)
			for label, q_goal, planning_time, execution_time in sequence:
				self.get_logger().info(
					f"Planning step: {label}, planning_time={planning_time:.1f}s, "
					f"execution_time={execution_time:.1f}s"
				)
				self._move_ompl(
					q_goal,
					planning_time=planning_time,
					execution_time=execution_time,
				)
		except Exception as exc:
			self.get_logger().error(f"Welcome motion failed: {exc}")


def main():
	rclpy.init()
	node = WelcomeController()
	try:
		node.run_welcome_sequence()
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
