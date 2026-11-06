from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from kaixuan_control_lab.run_torque_line import (
    DAMPING_RATIO, NATURAL_FREQUENCY_RAD_S, TARGET_JOINTS_DEG,
    computed_torque, draw_path, prepare, simulate,
)


class TorqueLineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.config, cls.plant, cls.dynamics, cls.reference = prepare(Path(cls.directory.name))
        cls.summary, cls.trace = simulate(cls.plant, cls.dynamics, cls.reference)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_world_vertical_path_and_fixed_tool_orientation(self):
        ref = self.reference
        np.testing.assert_allclose(ref.q[0], np.deg2rad(TARGET_JOINTS_DEG), atol=1e-12)
        np.testing.assert_allclose(ref.positions[-1] - ref.positions[0], [0, 0, .1], atol=1e-12)
        np.testing.assert_allclose(ref.positions[:, :2], np.tile(ref.start_position[:2], (len(ref.times), 1)), atol=1e-12)
        self.assertTrue(np.all(np.diff(ref.positions[:, 2]) >= -1e-14))
        self.assertLess(ref.diagnostics["max_position_error_m"], 1e-6)
        self.assertLess(ref.diagnostics["max_orientation_error_rad"], 1e-6)
        np.testing.assert_allclose(ref.dq[[0, -1]], 0, atol=1e-12)

    def test_measured_trajectory_and_motor_torque(self):
        summary, trace = self.summary, self.trace
        self.assertEqual(summary["status"], "PASS", summary)
        self.assertLess(summary["max_tracking_error_mm"], .01)
        self.assertLess(summary["peak_speed_deg_s"], 90)
        self.assertEqual(summary["torque_saturated_steps"], 0)
        np.testing.assert_allclose(summary["actual_displacement_world_m"], [0, 0, .1], atol=1e-7)
        # The motor generalized force is precisely the requested Nm, gear=1.
        np.testing.assert_allclose(trace["motor_generalized_nm"], trace["requested_nm"], atol=1e-12)
        self.assertTrue(np.any(np.abs(trace["q_rad"][-1]-trace["q_rad"][0]) > .1))

    def test_gravity_only_cannot_follow_the_moving_reference(self):
        # Same moving reference, but send gravity only: it must stay at rest.
        # This would catch a runner that writes reference q directly into qpos.
        self.plant.reset()
        with patch("kaixuan_control_lab.run_torque_line.computed_torque",
                   side_effect=lambda dynamics, q, dq, *unused: dynamics.gravity(q)):
            summary, trace = simulate(self.plant, self.dynamics, self.reference)
        np.testing.assert_allclose(trace["q_rad"], np.tile(np.deg2rad(TARGET_JOINTS_DEG), (len(trace["time_s"]), 1)), atol=1e-10)
        self.assertEqual(summary["status"], "FAIL")
        self.assertAlmostEqual(summary["final_error_mm"], 100., places=6)

    def test_computed_torque_recovers_expected_coupled_acceleration(self):
        class CoupledModel:
            def mass(self, q):
                return np.array([[2., .4], [.4, .7]])

            def bias(self, q, dq):
                return np.array([3*np.sin(q[0]) + dq[1]**2, .2*np.cos(q[1])])

        model = CoupledModel()
        q, dq = np.array([.3, -.2]), np.array([-.1, .2])
        qr, dqr, ddqr = q + [.01, -.02], dq + [.03, -.04], np.array([.1, .2])
        tau = computed_torque(model, q, dq, qr, dqr, ddqr)
        actual_acceleration = np.linalg.solve(model.mass(q), tau-model.bias(q, dq))
        expected = ddqr + NATURAL_FREQUENCY_RAD_S**2*(qr-q) + 2*DAMPING_RATIO*NATURAL_FREQUENCY_RAD_S*(dqr-dq)
        np.testing.assert_allclose(actual_acceleration, expected, atol=1e-12)

    def test_visual_markers_need_no_position_override_or_graphics_context(self):
        class Viewer:
            user_scn = mujoco.MjvScene(self.plant.model, maxgeom=16)

        viewer = Viewer()
        q_before = self.plant.state().q
        start = self.reference.start_position
        draw_path(viewer, start + [0, 0, .05], start + [0, 0, .049], start, start + [0, 0, .1])
        self.assertEqual(viewer.user_scn.ngeom, 4)
        np.testing.assert_array_equal(self.plant.state().q, q_before)


if __name__ == "__main__":
    unittest.main()
