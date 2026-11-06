import copy
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np
from scipy.linalg import block_diag
from scipy.signal import cont2discrete

from kaixuan_control_lab.backend import MujocoPlant
from kaixuan_control_lab.config import ROOT, load_config
from kaixuan_control_lab.lqr import (TrajectoryController, design_tvlqr, forward_dynamics,
                                    linearize_dynamics, state_weights)
from kaixuan_control_lab.lqr_experiment import disturbance_wrench, recovery_time, run_case
from kaixuan_control_lab.lqr_video import projection_basis
from kaixuan_control_lab.model import build_model
from kaixuan_control_lab.run_lqr import read_settings, synthesize
from kaixuan_control_lab.trajectory import build_reference, integer_steps, solve_ik


class CoupledAnalyticDynamics:
    m = np.array([[2., .6], [.6, 1.]])
    stiffness = np.array([[3., .2], [.2, 2.]])
    damping = np.diag([.3, .2])

    def mass(self, q):
        return self.m

    def bias(self, q, dq):
        return self.stiffness @ q + self.damping @ dq


class LQRMathTests(unittest.TestCase):
    def test_coupled_linearization_and_zoh_against_known_physics(self):
        dyn = CoupledAnalyticDynamics()
        a, b, ad, bd = linearize_dynamics(dyn, np.array([.2, -.1]), np.array([.1, .3]), np.array([.5, -.2]), .01)
        expected_a = np.block([[np.zeros((2, 2)), np.eye(2)],
            [-np.linalg.solve(dyn.m, dyn.stiffness), -np.linalg.solve(dyn.m, dyn.damping)]])
        expected_b = np.vstack((np.zeros((2, 2)), np.linalg.inv(dyn.m)))
        expected_ad, expected_bd, _, _, _ = cont2discrete((expected_a, expected_b, np.eye(4), np.zeros((4, 2))), .01)
        np.testing.assert_allclose(a, expected_a, atol=1e-10)
        np.testing.assert_allclose(b, expected_b, atol=1e-12)
        np.testing.assert_allclose(ad, expected_ad, atol=1e-11)
        np.testing.assert_allclose(bd, expected_bd, atol=1e-11)

    def test_backward_feedback_matches_independent_batch_optimum(self):
        # Solve the entire open-loop quadratic program by dense elimination,
        # independently of the backward Riccati recursion.
        horizon, n, m = 6, 4, 2
        rng = np.random.default_rng(8)
        ad = np.array([np.eye(n)+rng.normal(size=(n, n))*.03 for _ in range(horizon)])
        bd = np.array([np.vstack((.01*np.eye(m), .1*np.eye(m))) for _ in range(horizon)])
        q = np.array([np.diag([2., 3., 1., 1.])*(1+.1*k) for k in range(horizon)])
        r = np.diag([.2, .3])
        design = design_tvlqr(ad, bd, q, r)
        x0 = np.array([.2, -.3, .1, .4])
        s, t = [], []
        transition, influence = np.eye(n), np.zeros((n, horizon*m))
        for k in range(horizon):
            transition = ad[k] @ transition
            influence = ad[k] @ influence
            influence[:, k*m:(k+1)*m] += bd[k]
            s.append(transition.copy())
            t.append(influence.copy())
        s, t = np.vstack(s), np.vstack(t)
        weight = block_diag(*list(q[1:]), design.p[-1])
        hessian = block_diag(*([r]*horizon)) + t.T @ weight @ t
        optimum = -np.linalg.solve(hessian, t.T @ weight @ s @ x0)
        x, actual = x0.copy(), []
        for k in range(horizon):
            u = -design.gains[k] @ x
            actual.extend(u)
            x = ad[k] @ x + bd[k] @ u
        np.testing.assert_allclose(actual, optimum, atol=1e-10)

    def test_rejects_invalid_weights_and_timebase(self):
        ad = np.array([np.eye(2)])
        bd = np.array([np.eye(2)*.1])
        for q, r in ((np.array([np.diag([1., -1.])]), np.eye(2)),
                     (np.array([np.eye(2)]), np.diag([1., 0.])),
                     (np.array([np.eye(2)*np.nan]), np.eye(2))):
            with self.assertRaises(ValueError):
                design_tvlqr(ad, bd, q, r)
        for dt in (0., -.01, .0015):
            with self.assertRaises(ValueError):
                integer_steps(dt, .001, "control")

    def test_recovery_requires_remaining_run_to_stay_inside_band(self):
        t = np.arange(6)*.1
        self.assertAlmostEqual(recovery_time(t, np.array([0., .2, 0., .2, 0., 0.]), .15, .1), .25)
        self.assertIsNone(recovery_time(t, np.array([0., 0., 0., 0., 0., .2]), .15, .1))

    def test_smooth_pulse_has_expected_sign_support_and_impulse(self):
        for filename in ("cartesian_lqr.yaml", "cartesian_lqr_demo.yaml"):
            settings, _ = read_settings(ROOT / "config" / filename)
            pulse = settings["disturbance"]
            dt = .001
            duration = sum(settings["trajectory"][key] for key in ("start_hold_s", "move_s", "end_hold_s"))
            times = np.arange(0., duration, dt)
            forces = np.array([disturbance_wrench(t, settings, -1.)[:3] for t in times])
            np.testing.assert_allclose(np.sum(forces, axis=0)*dt,
                -.5*pulse["duration_s"]*np.asarray(pulse["force_world_n"]), atol=1e-12)
            np.testing.assert_allclose(forces[times < pulse["start_s"]], 0.)
            np.testing.assert_allclose(forces[times >= pulse["start_s"]+pulse["duration_s"]], 0.)

    def test_video_projection_preserves_physical_length_and_input_vector(self):
        delta = np.array([.24, .16, .08])
        original = delta.copy()
        basis = projection_basis(delta, np.array([0., 5., 0.]))
        np.testing.assert_array_equal(delta, original)
        np.testing.assert_allclose(basis @ basis.T, np.eye(2), atol=1e-12)
        np.testing.assert_allclose(basis @ delta, [np.linalg.norm(delta), 0.], atol=1e-12)
        self.assertGreater((basis @ np.array([0., 5., 0.]))[1], 0.)


class RobotExperimentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.settings, cls.config = read_settings(ROOT / "config/cartesian_lqr.yaml")
        cls.path = build_model(cls.config, cls.root / "robot.xml", visuals=False)
        cls.plant = MujocoPlant(cls.path, cls.config)
        cls.dyn = cls.plant.dynamics
        cls.reference = build_reference(cls.dyn, np.asarray(cls.config["initial_position_rad"]),
            *cls.plant.model.jnt_range.T, cls.settings, cls.config["physics_step_s"])
        cls.design = synthesize(cls.dyn, cls.reference, cls.settings)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_reference_is_a_line_with_fixed_orientation_and_multiple_moving_joints(self):
        diag = self.reference.diagnostics
        self.assertLess(diag["max_position_error_m"], 1e-6)
        self.assertLess(diag["max_orientation_error_rad"], 1e-6)
        self.assertGreaterEqual(sum(np.asarray(diag["joint_excursion_deg"]) > .1), 3)
        np.testing.assert_allclose(self.reference.dq[[0, -1]], 0., atol=1e-12)
        self.assertGreater(diag["min_6d_jacobian_singular_value_mixed_units"], .01)

    def test_external_force_maps_to_tool_origin_and_clears_on_next_step(self):
        plant = self.plant
        q0 = np.asarray(self.config["initial_position_rad"])
        dq0 = np.linspace(-.02, .02, 7)
        plant.reset(q=q0, dq=dq0)
        wrench = np.array([.1, .2, -.1, .01, .02, -.01])
        jac = self.dyn.kinematics(q0)[2]
        external = jac.T @ wrench
        # Independent MuJoCo force/torque application API verifies the moment arm.
        scratch = mujoco.MjData(plant.model)
        scratch.qpos[:] = q0
        mujoco.mj_forward(plant.model, scratch)
        separate = np.zeros(7)
        site = plant.model.site("ee_site").id
        mujoco.mj_applyFT(plant.model, scratch, wrench[:3], wrench[3:], scratch.site_xpos[site],
                         int(plant.model.site_bodyid[site]), separate)
        np.testing.assert_allclose(external, separate, atol=1e-12)
        self.assertAlmostEqual(float(external @ dq0), float(wrench @ (jac @ dq0)), places=13)
        torque = self.dyn.bias(q0, dq0)
        mass = self.dyn.mass(q0)
        plant.step(torque, external_wrench_world=wrench)
        np.testing.assert_allclose(plant.data.qfrc_applied, external, atol=1e-12)
        np.testing.assert_allclose(mass @ plant.data.qacc, external, atol=1e-9)
        plant.step(self.dyn.bias(plant.state().q, plant.state().dq))
        np.testing.assert_allclose(plant.data.qfrc_applied, 0.)

    def test_each_controller_adds_gravity_once_and_receives_no_push(self):
        index = 0
        for name in TrajectoryController.names:
            controller = TrajectoryController(name, self.dyn, self.reference, self.design, self.settings)
            torque = controller.compute(self.reference.q[index], self.reference.dq[index], index)
            np.testing.assert_allclose(torque, self.reference.tau[index], atol=1e-12)

    def test_differential_model_predicts_actual_nonlinear_acceleration(self):
        k = 190
        q, dq, tau = self.reference.q[k], self.reference.dq[k], self.reference.tau[k]
        x = np.r_[q, dq]
        self.plant.reset(q=q, dq=dq)
        self.plant.data.ctrl[:] = tau
        mujoco.mj_forward(self.plant.model, self.plant.data)
        np.testing.assert_allclose(forward_dynamics(self.dyn, x, tau)[7:], self.plant.data.qacc, atol=1e-9)
        self.assertLess(self.design.diagnostics["linearization_directional_relative_error"], 1e-4)

    def test_tvlqr_run_sampling_force_trace_and_recovery(self):
        controller = TrajectoryController("tvlqr", self.dyn, self.reference, self.design, self.settings)
        result, trace = run_case(self.root / "run", self.path, self.config, controller, self.settings,
                                 self.settings["scenarios"]["push"])
        self.assertEqual(result["status"], "completed")
        self.assertEqual(len(trace["time_s"]), 5001)
        self.assertEqual(result["control_updates"], 500)
        self.assertEqual(result["torque_saturated_steps"], 0)
        self.assertLess(result["final_position_error_mm"], 1.)
        self.assertIsNotNone(result["recovery_after_pulse_s"])
        for k in range(0, 5000, 10):
            np.testing.assert_allclose(trace["requested_nm"][k:k+10],
                np.broadcast_to(trace["requested_nm"][k], (10, 7)))
        self.assertTrue(np.any(trace["force_world_n"]))
        self.assertLess(self.design.diagnostics["terminal_spectral_radius"], 1.)

    def test_unreachable_ik_raises_and_never_supplies_an_action(self):
        with self.assertRaises(RuntimeError):
            solve_ik(self.dyn, np.array([10., 10., 10.]), self.reference.orientation,
                self.reference.q[0], *self.plant.model.jnt_range.T, max_iterations=3)


if __name__ == "__main__":
    unittest.main()
