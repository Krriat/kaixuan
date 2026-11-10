"""Numerical/guard regression checks; these tests never open EtherCAT or ROS."""
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/"scripts"))
from cst_cartesian_math import (JOINTS, RobotModel, TorqueFeedback, hardware_settings,
                                joint_reference, make_plan, rotation_error, validate_resume_pose)
from cst_cartesian_trajectory import cruise_progress


class CartesianTorqueTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load((PACKAGE/"config/cst_cartesian_test.yaml").read_text())
        xml = (PACKAGE.parent/"xiaogong_kaixuan/urdf/xiaogong-kaixuan.urdf").read_text()
        cls.model = RobotModel(xml, cls.config, np.zeros(7))
        raw = yaml.safe_load((PACKAGE.parent/"kaixuan_can_hardware/config/cartesian_torque_limits.yaml").read_text())["joints"]
        cls.limits = {k: np.array([raw[j][k] for j in JOINTS]) for k in (
            "min_position_rad", "max_position_rad", "effort_limit_nm", "effort_slew_nm_s",
            "max_velocity_rad_s", "max_displacement_rad")}
        cls.reference, cls.samples, cls.plan = make_plan(cls.model, np.zeros(7), cls.config, cls.limits)

    def test_cartesian_reference_is_vertical_and_holds_orientation(self):
        p0, r0, _ = self.model.kinematics(np.deg2rad(self.config["target_joints_deg"]))
        for t in np.linspace(0, self.reference.times[-1], 91):
            p, rotation, _ = self.model.kinematics(self.reference.sample(t)[0])
            np.testing.assert_allclose(p[:2], p0[:2], atol=1e-7)
            self.assertLess(np.linalg.norm(rotation_error(r0, rotation)), 1e-7)
        p1, _, _ = self.model.kinematics(self.reference.q[-1])
        np.testing.assert_allclose(p1-p0, [0, 0, .1], atol=1e-9)
        np.testing.assert_allclose(self.reference.dq[[0, -1]], 0, atol=1e-12)

    def test_approach_is_c2_at_both_endpoints(self):
        a, b = np.zeros(7), np.deg2rad(self.config["target_joints_deg"])
        for t, q in ((0., a), (20., b)):
            actual, velocity, acceleration = joint_reference(a, b, t, 20.)
            np.testing.assert_allclose(actual, q, atol=1e-14)
            np.testing.assert_array_equal(velocity, np.zeros(7))
            np.testing.assert_array_equal(acceleration, np.zeros(7))

    def test_cruise_profile_is_monotone_symmetric_and_has_smooth_velocity(self):
        t = np.linspace(-1., 11., 12001)
        p = cruise_progress(t, 10., 1.)
        self.assertTrue(np.all(np.diff(p) >= -1e-15))
        np.testing.assert_allclose(p+cruise_progress(10.-t, 10., 1.), 1., atol=1e-15)
        np.testing.assert_array_equal(cruise_progress([-2., 0., 10., 12.], 10., 1.), [0., 0., 1., 1.])
        v = np.gradient(p, .001)
        a = np.gradient(v, .001)
        np.testing.assert_allclose(v[(t>1.1)&(t<8.9)], 1./9., atol=1e-12)
        for junction in (0., 1., 9., 10.):
            self.assertLess(np.max(np.abs(a[np.abs(t-junction)<.002])), 2e-5)
        for ramp in (-1., 0., 5.01, float("nan")):
            with self.assertRaises(ValueError):
                cruise_progress(0., 10., ramp)

    def test_cruise_plan_preserves_geometry_and_obeys_reference_speed_limit(self):
        config = dict(self.config, line_profile="smooth_cruise", cruise_ramp_s=1., line_s=10.)
        reference, _, plan = make_plan(self.model, np.zeros(7), config, self.limits)
        self.assertEqual(plan["line"]["time_profile"], "smooth_cruise")
        for t in np.linspace(0., reference.times[-1], 401):
            q, _, _ = reference.sample(t)
            p, r, _ = self.model.kinematics(q)
            np.testing.assert_allclose(p, reference.desired_position(t), atol=1e-7)
            self.assertLess(np.linalg.norm(rotation_error(reference.orientation, r)), 1e-7)
        np.testing.assert_allclose(reference.dq[[0, -1]], 0., atol=1e-12)
        with self.assertRaisesRegex(ValueError, "too fast"):
            make_plan(self.model, np.zeros(7), dict(config, max_reference_speed_deg_s=.01), self.limits)

    def test_reference_rejects_insufficient_torque_margin(self):
        low_limits = dict(self.limits, effort_limit_nm=np.full(7, .01))
        with self.assertRaisesRegex(ValueError, "torque margin"):
            make_plan(self.model, np.zeros(7), self.config, low_limits)

    def test_rnea_adds_armature_once_and_gravity_once(self):
        q = np.deg2rad(self.config["target_joints_deg"])
        dq, ddq = np.arange(7)*.005, np.arange(7)*.01
        expected = self.model.mass(q)@ddq + self.model.bias(q, dq)
        np.testing.assert_allclose(self.model.inverse_dynamics(q, dq, ddq), expected, atol=1e-12)
        control = TorqueFeedback(self.model, self.config, self.limits)
        # Preload the previous output to avoid testing startup slew in this identity.
        control.previous = expected.copy()
        command, _, _ = control.step(q, dq, (q, dq, ddq), .005)
        np.testing.assert_allclose(command, expected, atol=1e-12)

    def test_saturation_slew_and_antiwindup_with_stuck_joint(self):
        control = TorqueFeedback(self.model, self.config, self.limits)
        q = np.zeros(7)
        last = np.zeros(7)
        for _ in range(400):
            # Keep every axis saturated with the raised 20 Nm output bounds.
            command, _, integral = control.step(q, q, (np.full(7, 1.), q, q), .005)
            self.assertTrue(np.all(np.abs(command) <= self.limits["effort_limit_nm"]+1e-12))
            self.assertTrue(np.all(np.abs(command-last) <= self.limits["effort_slew_nm_s"]*.005+1e-12))
            np.testing.assert_allclose(integral, 0, atol=1e-12)
            last = command

    def test_zero_gravity_ramp_cannot_produce_initial_torque(self):
        control = TorqueFeedback(self.model, self.config, self.limits)
        q = np.deg2rad(self.config["target_joints_deg"])
        command, _, integral = control.step(q, np.zeros(7), (q, np.zeros(7), np.zeros(7)), .005, scale=0.)
        np.testing.assert_array_equal(command, 0)
        np.testing.assert_array_equal(integral, 0)

    def test_mock_and_missing_hardware_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "no ros2_control"):
            hardware_settings('<robot name="test"/>')
        with self.assertRaisesRegex(ValueError, "real Kaixuan"):
            hardware_settings('<robot><ros2_control><hardware><plugin>mock_components/GenericSystem</plugin>'
                              '</hardware></ros2_control></robot>')

    def test_motion_profile_only_overrides_requested_effort_bounds(self):
        source = PACKAGE.parent/"kaixuan_can_hardware/config"
        gravity = yaml.safe_load((source/"gravity_limits.yaml").read_text())["joints"]
        motion = yaml.safe_load((source/"cartesian_torque_limits.yaml").read_text())["joints"]
        for name in gravity:
            for key in ("effort_limit_nm", "max_velocity_rad_s", "max_displacement_rad",
                        "min_position_rad", "max_position_rad", "torque_command_units_per_nm"):
                expected = 20.0 if name in ("left_joint1", "left_joint3", "left_joint4") and key == "effort_limit_nm" else gravity[name][key]
                self.assertAlmostEqual(expected, motion[name][key], places=12)

    def test_saved_stop_restart_requires_matching_stationary_disabled_pose(self):
        snapshot = {"q_deg": {j: 0. for j in JOINTS}, "velocity_deg_s": {j: 0. for j in JOINTS},
                    "diagnostics": {j: dict(control_word="6", status_word="561", effort_claimed="0",
                        torque_armed="0", error_code="0", torque_guard_fault="0", target_torque_raw="0") for j in JOINTS}}
        np.testing.assert_array_equal(validate_resume_pose(snapshot, np.zeros(7), np.zeros(7)), np.zeros(7))
        for d in snapshot["diagnostics"].values():
            d.update(control_word="0", status_word="592")
        np.testing.assert_array_equal(validate_resume_pose(snapshot, np.zeros(7), np.zeros(7)), np.zeros(7))
        snapshot["diagnostics"][JOINTS[0]]["status_word"] = "563"  # switched on, not a valid saved stop
        with self.assertRaises(ValueError):
            validate_resume_pose(snapshot, np.zeros(7), np.zeros(7))
        snapshot["diagnostics"][JOINTS[0]]["status_word"] = "592"
        with self.assertRaisesRegex(ValueError, "does not match"):
            validate_resume_pose(snapshot, np.deg2rad(np.full(7, .3)), np.zeros(7))
        with self.assertRaisesRegex(ValueError, "stationary"):
            validate_resume_pose(snapshot, np.zeros(7), np.deg2rad(np.full(7, .2)))
        snapshot["diagnostics"][JOINTS[3]]["torque_armed"] = "1"
        with self.assertRaisesRegex(ValueError, "clean disabled"):
            validate_resume_pose(snapshot, np.zeros(7), np.zeros(7))

if __name__ == "__main__":
    unittest.main()
