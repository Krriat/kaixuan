#!/usr/bin/env python3

import time

import rclpy

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK
from moveit_msgs.action import ExecuteTrajectory


class CommonlyFunctionLeft:
	MOVEIT_ERROR_TEXT = {
		1: "SUCCESS",
		99999: "FAILURE",
		-1: "PLANNING_FAILED",
		-2: "INVALID_MOTION_PLAN",
		-3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
		-4: "CONTROL_FAILED",
		-5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
		-6: "TIMED_OUT",
		-7: "PREEMPTED",
		-10: "START_STATE_IN_COLLISION",
		-11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
		-12: "GOAL_IN_COLLISION",
		-13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
		-14: "GOAL_CONSTRAINTS_VIOLATED",
		-15: "INVALID_GROUP_NAME",
		-16: "INVALID_GOAL_CONSTRAINTS",
		-17: "INVALID_ROBOT_STATE",
		-18: "INVALID_LINK_NAME",
		-19: "INVALID_OBJECT_NAME",
		-21: "FRAME_TRANSFORM_FAILURE",
		-22: "COLLISION_CHECKING_UNAVAILABLE",
		-23: "ROBOT_STATE_STALE",
		-24: "SENSOR_INFO_STALE",
		-31: "NO_IK_SOLUTION",
	}

	OMPL_MAX_VELOCITY_SCALING = 0.8
	OMPL_MAX_ACCELERATION_SCALING = 0.8

	GROUP_CONFIG = {
		"left_arm": {
			"joint_names": [
				"left_joint1",
				"left_joint2",
				"left_joint3",
				"left_joint4",
				"left_joint5",
				"left_joint6",
				"left_joint7",
			],
			"ee_link": "left_joint7",
			"base_frame": "base_link",
			"cartesian_max_step": 0.005,
			"cartesian_target_duration": None,
		}
	}

	OPEN_CMD = "255,255,255,255,255,255"
	HALF_GRASP_CMD = "255,255,80,80,120,200"
	GRASP_CMD = "255,255,40,40,40,40"
	HOLD_CMD = "255,255,5,15,20,20"
	XIAOZHUA_CMD="255,255,220,220,255,255"

	def _ensure_group_state_cache(self):
		if not hasattr(self, "_group_last_q"):
			self._group_last_q = {}

	def _get_group_config(self, group_name):
		cfg = self.GROUP_CONFIG.get(group_name)
		if cfg is None:
			raise RuntimeError(f"Unsupported group_name: {group_name}")
		return cfg

	def get_group_joint_names(self, group_name):
		return list(self._get_group_config(group_name)["joint_names"])

	def set_group_last_q(self, group_name, q_value):
		self._ensure_group_state_cache()
		self._group_last_q[group_name] = [float(x) for x in q_value]

	def get_group_last_q(self, group_name):
		self._ensure_group_state_cache()
		value = self._group_last_q.get(group_name)
		if value is None:
			return None
		return list(value)

	def execute_ompl_goal(self, q_goal, planning_time, execution_time, group_name):
		traj, q_end = self.plan_ompl_goal(q_goal, planning_time, execution_time, group_name)
		result_future = self.start_execute_traj(traj)
		self.finish_execute_traj(result_future)
		self.set_group_last_q(group_name, q_end)
		return q_end

	def plan_ompl_goal(self, q_goal, planning_time, execution_time, group_name):
		joint_names = self.get_group_joint_names(group_name)
		current_q = self.get_current_joint_positions(group_name, timeout_sec=1.0)

		if current_q is not None:
			attempts = [
				(current_q, "explicit_current_state"),
				(None, "moveit_current_state"),
			]
		else:
			attempts = [
				(None, "moveit_current_state"),
			]

		last_error = None
		for idx, (q_start, start_mode) in enumerate(attempts, start=1):
			try:
				self.get_logger().info(f"OMPL wrapper attempt {idx}: start={start_mode}")
				traj, q_end = self.plan_to_joint_ompl(
					group_name,
					joint_names,
					q_goal,
					planning_time,
					execution_time,
					goal_name="goal",
					q_start=q_start,
				)
				return traj, q_end
			except RuntimeError as exc:
				last_error = exc
				self.get_logger().warn(f"OMPL wrapper attempt {idx} failed: {exc}")

		raise last_error

	def execute_cartesian_goal(self, q_goal, group_name):
		traj, q_end = self.plan_cartesian_goal(q_goal, group_name)
		result_future = self.start_execute_traj(traj)
		self.finish_execute_traj(result_future)
		self.set_group_last_q(group_name, q_end)
		return q_end

	def plan_cartesian_goal(self, q_goal, group_name):
		cfg = self._get_group_config(group_name)
		joint_names = list(cfg["joint_names"])
		ee_link = cfg["ee_link"]
		base_frame = cfg["base_frame"]
		max_step = float(cfg.get("cartesian_max_step", 0.005))
		target_duration = cfg.get("cartesian_target_duration")

		q_start = self.get_current_joint_positions(group_name, timeout_sec=1.0)
		if q_start is None:
			raise RuntimeError(f"Cannot get current joint positions for group: {group_name}")
		self.get_logger().info("Cartesian wrapper: q_start=current_state")

		traj = self.compute_cartesian(
			group_name,
			ee_link,
			base_frame,
			joint_names,
			q_start,
			q_goal,
			max_step,
		)
		traj = self.retime_trajectory(traj, target_duration=target_duration)

		q_end = self.extract_last_joint_positions(traj, joint_names, q_start)
		return traj, q_end

	def setup_hand_publisher(self, topic="/left_hand_action", queue_size=10):
		self.hand_pub = self.create_publisher(String, topic, queue_size)

	def setup_joint_state_subscriber(self, topic="/left/joint_states"):
		if hasattr(self, "_joint_state_sub") and self._joint_state_sub is not None:
			return
		self._latest_joint_state = None
		self._joint_state_sub = self.create_subscription(
			JointState,
			topic,
			self._on_joint_state,
			10,
		)

	def _on_joint_state(self, msg):
		self._latest_joint_state = msg

	def get_current_joint_positions(self, group_name, timeout_sec=1.0):
		self.setup_joint_state_subscriber("/left/joint_states")
		joint_names = self.get_group_joint_names(group_name)

		deadline = time.time() + max(0.0, float(timeout_sec))
		while time.time() <= deadline:
			msg = getattr(self, "_latest_joint_state", None)
			if msg is not None:
				name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
				if all(n in name_to_pos for n in joint_names):
					return [float(name_to_pos[n]) for n in joint_names]
			rclpy.spin_once(self, timeout_sec=0.05)

		return None

	def publish_hand(self, cmd):
		msg = String()
		msg.data = cmd
		self.hand_pub.publish(msg)

	def hand_open(self, wait_sec=0.0):
		self.publish_hand(self.OPEN_CMD)
		self.get_logger().info(f"OPEN -> {self.OPEN_CMD}")
		if wait_sec > 0.0:
			time.sleep(wait_sec)

	def hand_half_grasp(self, wait_sec=0.0):
		self.publish_hand(self.HALF_GRASP_CMD)
		self.get_logger().info(f"HALF_GRASP -> {self.HALF_GRASP_CMD}")
		if wait_sec > 0.0:
			time.sleep(wait_sec)

	def hand_grasp(self, wait_sec=0.0):
		self.publish_hand(self.GRASP_CMD)
		self.get_logger().info(f"GRASP -> {self.GRASP_CMD}")
		if wait_sec > 0.0:
			time.sleep(wait_sec)

	def hand_xiaozhua(self, wait_sec=0.0):
		self.publish_hand(self.XIAOZHUA_CMD)
		self.get_logger().info(f"GRASP -> {self.XIAOZHUA_CMD}")
		if wait_sec > 0.0:
			time.sleep(wait_sec)


	def hand_hold(self, wait_sec=0.0):
		self.publish_hand(self.HOLD_CMD)
		self.get_logger().info(f"HOLD -> {self.HOLD_CMD}")
		if wait_sec > 0.0:
			time.sleep(wait_sec)

	def _set_duration(self, point, t_sec):
		value = max(0.0, float(t_sec))
		sec = int(value)
		nanosec = int(round((value - sec) * 1e9))
		if nanosec >= 1000000000:
			sec += 1
			nanosec -= 1000000000
		point.time_from_start.sec = sec
		point.time_from_start.nanosec = nanosec

	def _duration_to_float(self, duration):
		return float(duration.sec) + float(duration.nanosec) * 1e-9

	def retime_trajectory(self, traj, target_duration=None):
		jt = traj.joint_trajectory
		if not jt.points:
			return traj

		n_points = len(jt.points)
		old_times = [self._duration_to_float(p.time_from_start) for p in jt.points]
		old_total = old_times[-1]

		if target_duration is None:
			return traj

		new_total = float(target_duration)
		if new_total <= 0.0:
			raise RuntimeError("target duration must be > 0")

		if n_points == 1:
			self._set_duration(jt.points[0], 0.0)
			return traj

		if old_total > 1e-9:
			time_scale = new_total / old_total
			for point, old_t in zip(jt.points, old_times):
				self._set_duration(point, old_t * time_scale)
				if point.velocities:
					point.velocities = [float(v) / time_scale for v in point.velocities]
				if point.accelerations:
					point.accelerations = [float(a) / (time_scale * time_scale) for a in point.accelerations]
		else:
			for i, point in enumerate(jt.points):
				self._set_duration(point, new_total * float(i) / float(n_points - 1))

		return traj

	def ensure_min_trajectory_duration(self, traj, min_duration):
		jt = traj.joint_trajectory
		if not jt.points:
			return traj

		current_total = self._duration_to_float(jt.points[-1].time_from_start)
		target_duration = float(min_duration)
		if target_duration <= current_total + 1e-6:
			return traj

		self.get_logger().info(
			f"Stretch OMPL trajectory duration from {current_total:.3f}s to {target_duration:.3f}s"
		)
		return self.retime_trajectory(traj, target_duration=target_duration)

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
		self.get_logger().info(f"IK error_code: {code}")
		if code != 1:
			raise RuntimeError(f"IK failed (error_code={code})")

		name_to_pos = {n: p for n, p in zip(res.solution.joint_state.name, res.solution.joint_state.position)}
		q_sol = []
		for n in joint_names:
			if n not in name_to_pos:
				raise RuntimeError(f"IK solution missing joint: {n}")
			q_sol.append(float(name_to_pos[n]))

		return q_sol

	def extract_last_joint_positions(self, traj, joint_names, fallback):
		q_last = list(fallback)
		jt = traj.joint_trajectory
		if jt.joint_names and jt.points:
			last = jt.points[-1].positions
			idx = {n: i for i, n in enumerate(jt.joint_names)}
			for i, name in enumerate(joint_names):
				j = idx.get(name)
				if j is not None and j < len(last):
					q_last[i] = float(last[j])
		return q_last

	def plan_to_joint_ompl(self, group_name, joint_names, q_goal, planning_time, execution_time, goal_name="goal", q_start=None):
		if not self.plan_cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError("/plan_kinematic_path service not available")

		req = GetMotionPlan.Request()
		mpr = req.motion_plan_request
		mpr.group_name = group_name
		mpr.pipeline_id = "ompl"
		mpr.num_planning_attempts = 5
		mpr.allowed_planning_time = max(5.0, float(planning_time))
		mpr.max_velocity_scaling_factor = float(self.OMPL_MAX_VELOCITY_SCALING)
		mpr.max_acceleration_scaling_factor = float(self.OMPL_MAX_ACCELERATION_SCALING)
		if q_start is not None:
			mpr.start_state.joint_state.name = list(joint_names)
			mpr.start_state.joint_state.position = [float(x) for x in q_start]

		goal_constraints = Constraints()
		goal_constraints.name = goal_name
		for name, pos in zip(joint_names, q_goal):
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
		self.get_logger().info(f"OMPL {goal_name} error_code: {code}")
		if code != 1:
			raise RuntimeError(f"OMPL planning {goal_name} failed (error_code={code})")

		traj = res.motion_plan_response.trajectory
		traj = self.ensure_min_trajectory_duration(traj, execution_time)
		q_end = self.extract_last_joint_positions(traj, joint_names, q_goal)
		return traj, q_end

	def compute_cartesian(self, group_name, ee_link, base_frame, joint_names, q_start, target_pose, max_step):
		if not self.cart_cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError("/compute_cartesian_path service not available")

		req = GetCartesianPath.Request()
		req.header.frame_id = base_frame
		req.group_name = group_name
		req.link_name = ee_link

		js = JointState()
		js.name = list(joint_names)
		js.position = list(q_start)
		req.start_state.joint_state = js

		req.waypoints = [target_pose]
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
		result_future = self.start_execute_traj(traj)
		self.finish_execute_traj(result_future)

	def start_execute_traj(self, traj):
		if not self.exec_cli.wait_for_server(timeout_sec=5.0):
			raise RuntimeError("/execute_trajectory action server not available")

		goal = ExecuteTrajectory.Goal()
		goal.trajectory = traj

		gh_future = self.exec_cli.send_goal_async(goal)
		rclpy.spin_until_future_complete(self, gh_future)
		gh = gh_future.result()
		if gh is None or not gh.accepted:
			raise RuntimeError("ExecuteTrajectory goal rejected")

		return gh.get_result_async()

	def finish_execute_traj(self, res_future):
		rclpy.spin_until_future_complete(self, res_future, timeout_sec=120.0)
		if not res_future.done():
			raise RuntimeError(
				"ExecuteTrajectory timed out after 120.0s; likely controller/drive stall or blocked execution"
			)
		res = res_future.result().result
		code = res.error_code.val
		code_text = self.MOVEIT_ERROR_TEXT.get(code, f"UNKNOWN({code})")
		self.get_logger().info(f"ExecuteTrajectory result error_code: {code} ({code_text})")
		if code != 1:
			if code == -7:
				raise RuntimeError(
					"ExecuteTrajectory failed (error_code=-7 PREEMPTED). "
					"Another node likely sent a new motion goal and interrupted this trajectory."
				)
			raise RuntimeError(f"ExecuteTrajectory failed (error_code={code} {code_text})")
