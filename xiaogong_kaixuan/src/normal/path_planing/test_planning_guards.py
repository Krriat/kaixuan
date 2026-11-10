#!/usr/bin/env python3
"""Offline checks for trajectory execution guards; no ROS graph is required."""

import contextlib
import io
import copy
import importlib
import math
import os
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes, RobotState, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath
from trajectory_msgs.msg import JointTrajectoryPoint

from planning_common import (
    JOINTS, LeftArmPlanner, cartesian_waypoints, circ_interim, duration_seconds,
    hardware_execution_error, parse_args, ros_environment_text, run, set_duration,
    stretch_trajectory, target_pose, validate_trajectory,
)


def example_parameters(module):
    parameters = copy.deepcopy(importlib.import_module(module).PARAMETERS)
    if module.startswith("ompl_"):
        parameters["joint_goal_deg"] = [0.0] * 7  # Offline test fixture only.
    return parameters


def trajectory():
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(JOINTS)
    result.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[0.0] * 7, velocities=[0.0] * 7, accelerations=[0.0] * 7),
        JointTrajectoryPoint(positions=[0.01] * 7, velocities=[0.02] * 7, accelerations=[0.1] * 7),
    ]
    set_duration(result.joint_trajectory.points[-1].time_from_start, 0.5)
    return result


class PlanningGuardsTest(unittest.TestCase):
    @staticmethod
    def hardware_probe(availability):
        client = Mock(srv_name="/controller_manager/list_hardware_components")
        client.wait_for_service.side_effect = availability
        response = SimpleNamespace(component=[SimpleNamespace(
            name="MockSystem", class_type="mock_components/GenericSystem")])
        return SimpleNamespace(real_ethercat_hardware=None, hardware_client=client,
                               wait_result=Mock(return_value=response), get_logger=Mock())

    def test_delayed_service_discovery_can_complete_before_execution(self):
        fake = self.hardware_probe([False, True])
        LeftArmPlanner.check_hardware_ready(fake)
        self.assertEqual([call.kwargs["timeout_sec"]
                          for call in fake.hardware_client.wait_for_service.call_args_list], [1.0, 14.0])
        fake.hardware_client.call_async.assert_called_once()
        self.assertFalse(fake.real_ethercat_hardware)

    def test_missing_service_reports_domain_and_does_not_skip_hardware_check(self):
        fake = self.hardware_probe([False, False])
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "0"}):
            with self.assertRaisesRegex(RuntimeError, r"list_hardware_components.*ROS_DOMAIN_ID=0.*bringup"):
                LeftArmPlanner.check_hardware_ready(fake)
        fake.hardware_client.call_async.assert_not_called()
        self.assertIsNone(fake.real_ethercat_hardware)

    def test_unset_domain_is_reported_as_ros_default_zero(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertIn("ROS_DOMAIN_ID=0 (unset; ROS default)", ros_environment_text())

    @staticmethod
    def healthy_hardware():
        states = {"ethercat/master1": {"dc_follower_failed": "0", "dc_reset_failed": "0",
                                       "wc_state": "2", "startup_ready": "1"}}
        states.update({name: {"status_word": "39", "error_code": "0", "control_word": "15"}
                       for name in JOINTS})
        return states

    def test_healthy_enabled_hardware_allows_execution(self):
        self.assertIsNone(hardware_execution_error(self.healthy_hardware()))

    def test_clock_failure_is_reported_even_when_bus_counter_is_complete(self):
        states = self.healthy_hardware()
        states["ethercat/master1"].update(dc_follower_failed="1", dc_follower_error="7")
        self.assertIn("dc_follower_error=7", hardware_execution_error(states))

    def test_inactive_startup_qualification_blocks_execution(self):
        states = self.healthy_hardware()
        states["ethercat/master1"]["startup_ready"] = "0"
        self.assertIn("not ready", hardware_execution_error(states))

    def test_operation_disabled_drive_is_identified(self):
        states = self.healthy_hardware()
        states["left_joint1"].update(status_word="561", control_word="6")
        reason = hardware_execution_error(states)
        self.assertIn("left_joint1", reason)
        self.assertIn("not operation-enabled", reason)

    def test_missing_drive_diagnostics_blocks_execution(self):
        states = self.healthy_hardware()
        del states["left_joint4"]
        self.assertIn("Missing drive diagnostics for left_joint4", hardware_execution_error(states))

    def test_hardware_failure_stops_before_sending_an_action_goal(self):
        fake = SimpleNamespace(check_hardware_ready=Mock(side_effect=RuntimeError("DC follower failed")),
                               execute_client=Mock())
        with self.assertRaisesRegex(RuntimeError, "DC follower failed"):
            LeftArmPlanner.execute(fake, trajectory(), RobotState())
        fake.execute_client.send_goal_async.assert_not_called()

    def test_rejects_right_arm_and_partial_left_arm(self):
        for names in ([f"right_joint{i}" for i in range(1, 8)], JOINTS[:-1]):
            with self.subTest(names=names):
                path = trajectory()
                path.joint_trajectory.joint_names = list(names)
                with self.assertRaises(RuntimeError):
                    validate_trajectory(path)

    def test_rejects_zero_or_decreasing_timestamps(self):
        path = trajectory()
        set_duration(path.joint_trajectory.points[-1].time_from_start, 0.0)
        with self.assertRaisesRegex(RuntimeError, "timestamps"):
            validate_trajectory(path)

    def test_rejects_nonfinite_positions(self):
        path = trajectory()
        path.joint_trajectory.points[-1].positions[0] = float("nan")
        with self.assertRaisesRegex(RuntimeError, "values"):
            validate_trajectory(path)

    def test_time_stretch_preserves_derivative_scaling(self):
        path = trajectory()
        stretch_trajectory(path, 4.0)
        self.assertEqual(validate_trajectory(path), 2.0)
        point = path.joint_trajectory.points[-1]
        self.assertEqual(duration_seconds(point.time_from_start), 2.0)
        self.assertAlmostEqual(point.velocities[0], 0.005)
        self.assertAlmostEqual(point.accelerations[0], 0.00625)
        self.assertEqual(point.positions[0], 0.01)

    def test_partial_cartesian_path_is_rejected_even_with_success_code(self):
        response = GetCartesianPath.Response()
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.fraction = 0.99
        response.solution = trajectory()
        fake = SimpleNamespace(cartesian_client=None, call=lambda *args: response)
        args, _ = parse_args("cartesian", importlib.import_module("cartesian_path").PARAMETERS, [])
        with self.assertRaisesRegex(RuntimeError, "incomplete"):
            LeftArmPlanner.plan_cartesian(fake, RobotState(), [Pose()], args)

    def test_each_python_file_selects_one_algorithm_and_executes_by_default(self):
        entries = {"ompl_rrtstar": "rrtstar", "ompl_rrtconnect": "rrtconnect",
                   "ompl_prm": "prm", "ompl_est": "est", "pilz_circ": "circ",
                   "cartesian_path": "cartesian", "pilz_lin": "lin"}
        for module, planner in entries.items():
            args, selected = parse_args(planner, example_parameters(module), [])
            self.assertEqual(selected, [planner])
            self.assertTrue(args.execute)
            self.assertTrue(args.return_to_start)

    def test_plan_only_disables_execution_and_return(self):
        args, _ = parse_args("rrtstar", example_parameters("ompl_rrtstar"), ["--plan-only"])
        self.assertFalse(args.execute)
        self.assertFalse(args.return_to_start)

    def test_repeated_execution_requires_return(self):
        parameters = example_parameters("ompl_rrtconnect")
        parameters.update(repeat=2, return_to_start=False)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                parse_args("rrtconnect", parameters, [])
        self.assertEqual(result.exception.code, 2)

    def test_python_inputs_are_used_without_affecting_another_example(self):
        original = importlib.import_module("ompl_rrtconnect").PARAMETERS
        parameters = copy.deepcopy(original)
        parameters["joint_goal_deg"] = [0, 0, 0, 10, 0, 0, 0]
        parameters["planner_parameters"]["range"] = 0.15
        args, _ = parse_args("rrtconnect", parameters, [])
        self.assertEqual(args.joint_goal_deg, [0, 0, 0, 10, 0, 0, 0])
        self.assertEqual(args.ompl_parameters["rrtconnect"]["range"], 0.15)
        self.assertEqual(original["planner_parameters"]["range"], 0.0)
        self.assertNotIn("planner_parameters", importlib.import_module("pilz_lin").PARAMETERS)

    def test_missing_ompl_joint_goal_stops_before_ros_initialization(self):
        for planner in ("rrtstar", "rrtconnect", "prm", "est"):
            parameters = example_parameters(f"ompl_{planner}")
            parameters["joint_goal_deg"] = None
            with self.subTest(planner=planner), patch("planning_common.rclpy.init") as init:
                with contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as result:
                        run(planner, parameters, argv=[])
                self.assertEqual(result.exception.code, 2)
                init.assert_not_called()

    def test_ompl_joint_goal_requires_seven_finite_angles(self):
        for goal in ([0.0] * 6, [float("nan")] + [0.0] * 6, ["0"] + [0.0] * 6):
            parameters = example_parameters("ompl_rrtconnect")
            parameters["joint_goal_deg"] = goal
            with self.subTest(goal=goal), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    parse_args("rrtconnect", parameters, [])
                self.assertEqual(result.exception.code, 2)

    def test_ompl_rejects_pose_inputs_instead_of_falling_back(self):
        for key in ("delta", "target_position_m", "target_rpy_deg"):
            parameters = example_parameters("ompl_rrtconnect")
            parameters[key] = [0.0, 0.0, 0.02]
            with self.subTest(key=key), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    parse_args("rrtconnect", parameters, [])
                self.assertEqual(result.exception.code, 2)

    def test_ompl_files_expose_only_joint_goal_variables(self):
        for planner in ("rrtstar", "rrtconnect", "prm", "est"):
            module = importlib.import_module(f"ompl_{planner}")
            self.assertTrue(hasattr(module, "TARGET_JOINTS_DEG"))
            for name in ("TARGET_OFFSET_M", "TARGET_POSITION_M", "TARGET_RPY_DEG"):
                self.assertFalse(hasattr(module, name))

    def test_cartesian_offsets_are_relative_to_start_and_preserve_order(self):
        parameters = copy.deepcopy(importlib.import_module("cartesian_path").PARAMETERS)
        parameters["waypoint_offsets_m"] = [[0, 0, 0.02], [0.005, 0, 0.02]]
        args, _ = parse_args("cartesian", parameters, [])
        start = target_pose(Pose(), position=[1, 2, 3])
        points = cartesian_waypoints(start, args)
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0].position.z, 3.02)
        self.assertAlmostEqual(points[1].position.z, 3.02)
        self.assertAlmostEqual(points[1].position.x, 1.005)

    def test_absolute_pose_and_rpy_override_relative_offset(self):
        pose = target_pose(Pose(), position=[1, 2, 3], offset=[9, 9, 9], rpy_deg=[0, 0, 90])
        self.assertEqual(pose.position.x, 1.0)
        self.assertAlmostEqual(pose.orientation.z, math.sqrt(0.5))
        self.assertAlmostEqual(pose.orientation.w, math.sqrt(0.5))

    def test_circ_explicit_via_point_and_collinearity(self):
        parameters = copy.deepcopy(importlib.import_module("pilz_circ").PARAMETERS)
        parameters["arc_position_m"] = [0, 0.003, 0.01]
        args, _ = parse_args("circ", parameters, [])
        start = Pose()
        goal = target_pose(start, offset=[0, 0, 0.02])
        self.assertAlmostEqual(circ_interim(start, goal, args).position.y, 0.003)
        args.arc_position_m = [0, 0, 0.01]
        with self.assertRaisesRegex(RuntimeError, "collinear"):
            circ_interim(start, goal, args)

    def test_unregistered_planner_cannot_silently_fall_back(self):
        fake = SimpleNamespace(planner_interfaces=[SimpleNamespace(
            pipeline_id="ompl", planner_ids=["left_arm[RRTConnect]"])])
        with self.assertRaisesRegex(RuntimeError, "not registered"):
            LeftArmPlanner.require_planner(fake, "ompl", "EST")


if __name__ == "__main__":
    unittest.main()
