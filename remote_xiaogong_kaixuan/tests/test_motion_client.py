"""Goal construction and rejection checks. No action goals are sent by these tests."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from motion_client import ArmClient, make_goal


class MotionGoalTests(unittest.TestCase):
    def test_default_is_left_group_planning_only_and_uses_current_server_state(self):
        goal = make_goal("left", [0.0] * 7)
        self.assertTrue(goal.planning_options.plan_only)
        self.assertTrue(goal.request.start_state.is_diff)
        self.assertEqual(list(goal.request.start_state.joint_state.position), [])
        self.assertTrue(goal.planning_options.planning_scene_diff.is_diff)
        self.assertTrue(goal.planning_options.planning_scene_diff.robot_state.is_diff)
        self.assertFalse(goal.planning_options.replan)
        self.assertEqual(goal.request.group_name, "left_arm")
        self.assertEqual(goal.request.max_velocity_scaling_factor, 0.1)
        constraints = goal.request.goal_constraints[0].joint_constraints
        self.assertEqual([c.joint_name for c in constraints], [f"left_joint{i}" for i in range(1, 8)])

    def test_execution_is_explicit(self):
        self.assertFalse(make_goal("left", [0.0] * 7, execute=True).planning_options.plan_only)
        self.assertTrue(make_goal("left", [0.0] * 7, execute=False).planning_options.plan_only)

    def test_invalid_length_nonfinite_values_and_limits_are_rejected(self):
        for angles in ([0] * 6, [0] * 8, [math.nan] * 7, [math.inf] * 7, [math.pi] * 7):
            with self.subTest(angles=angles), self.assertRaises(ValueError):
                make_goal("left", angles)

    def test_invalid_scaling_cannot_fall_back_to_full_speed(self):
        for key in ("velocity", "acceleration", "planning_time"):
            for value in (0, -0.1, math.inf, math.nan):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    make_goal("left", [0] * 7, **{key: value})
        with self.assertRaises(ValueError):
            make_goal("left", [0] * 7, velocity=1.01)

    def test_only_encoder_rounding_at_limit_is_clamped(self):
        goal = make_goal("left", [-3.14 - 1e-8] + [0.0] * 6)
        self.assertEqual(goal.request.goal_constraints[0].joint_constraints[0].position, -3.14)
        with self.assertRaises(ValueError):
            make_goal("left", [-3.14 - 1e-4] + [0.0] * 6)

    def test_robot_check_failure_sends_no_goal_even_with_execute(self):
        node = object.__new__(ArmClient)
        node.arm = "left"
        node._check_robot = Mock(side_effect=RuntimeError("model/controller mismatch"))
        node._move = Mock()
        with self.assertRaisesRegex(RuntimeError, "mismatch"):
            node.move_joints([0.0] * 7, execute=True)
        node._move.send_goal_async.assert_not_called()

    def test_unfinished_goal_cannot_be_overwritten_after_timeout(self):
        node = object.__new__(ArmClient)
        node._motion_pending = True
        node._result = Mock()
        node._result.done.return_value = False
        node._move = Mock()
        with self.assertRaisesRegex(RuntimeError, "上一个目标"):
            node.move_joints([0.0] * 7, execute=True)
        node._move.send_goal_async.assert_not_called()

    def test_relative_joint_goal_uses_feedback_and_preserves_other_targets(self):
        node = object.__new__(ArmClient)
        node.current_joint_angles = Mock(return_value=[0.1] * 7)
        node.move_joints = Mock()
        node.jog_joint(4, 2.0, execute=False)
        target = node.move_joints.call_args.args[0]
        self.assertAlmostEqual(target[3], 0.1 + math.radians(2))
        self.assertEqual(target[:3] + target[4:], [0.1] * 6)
        self.assertFalse(node.move_joints.call_args.kwargs["execute"])

    def cancellation_node(self):
        node = object.__new__(ArmClient)
        node._context = Mock()
        node._context.ok.return_value = True
        node._pending_goal = None
        node._result = None
        node._handle = Mock(accepted=True)
        node._cancel_requested = False
        node.get_logger = Mock()
        return node

    def test_cancellation_acknowledgement_is_not_sent_twice(self):
        node = self.cancellation_node()
        node._wait = Mock(return_value=SimpleNamespace(goals_canceling=[object()]))
        self.assertTrue(node.cancel())
        self.assertTrue(node.cancel())
        node._handle.cancel_goal_async.assert_called_once()

    def test_missing_cancellation_ack_is_reported_as_unknown_not_stopped(self):
        node = self.cancellation_node()
        node._wait = Mock(side_effect=RuntimeError("cancel timed out"))
        self.assertFalse(node.cancel())
        self.assertFalse(node._cancel_requested)
        node.get_logger().error.assert_called_once()
        self.assertIn("可能继续原轨迹", node.get_logger().error.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
