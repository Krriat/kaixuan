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

try:
	from tts import play_tts
except ModuleNotFoundError:
	sys.path.append(str(Path(__file__).resolve().parent))
	from tts import play_tts


class WelcomeController(Node, CommonlyFunction):
	def __init__(self):
		super().__init__("welcome_controller")

		self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
		self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
		self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
		self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

		self.group_name = "right_arm"
		self.q_a = [1.256, -0.105, 0.506, 1.868, 0.0, 0.0, -1.082]
		self.q_b = [1.256, -0.105, -0.506, 1.868, 0.0, 0.0, -1.082]
		self.q_2 = [1.0, -0.140510, -0.620728, 0.605678, -1.576051, -0.182540, 2.020647]
		self.q_3 = [angle_deg / 180.0 * math.pi for angle_deg in [17, -24, 57, 50, -57, 6, 73]]
		self.q_4 = [0,0,0,0,0,0,0]

		self.get_logger().info("Started. Execute welcome motion directly.")

	def _move_ompl(self, q_goal, move_time=2.0):
		self.execute_ompl_goal(q_goal=q_goal, move_time=move_time, group_name=self.group_name)

	def _speak_welcome(self):
		play_tts("您好，欢迎主管莅临制一处激励动员大会")

	def _speak_welcome_async(self):
		threading.Thread(target=self._speak_welcome, daemon=True).start()

	def run_welcome_sequence(self):
		sequence = [
			# ("q_a", self.q_a),
			("q_b", self.q_b),
			("q_a", self.q_a),
			("q_b", self.q_b),
			# ("q_2", self.q_2),
			# ("q_3", self.q_3),
			("q_4", self.q_4),
		]
		try:
			self.get_logger().info("Execute A -> B -> A -> target")
			self._speak_welcome_async()
			for label, q_goal in sequence:
				self.get_logger().info(f"Planning step: {label}")
				self._move_ompl(q_goal)
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
