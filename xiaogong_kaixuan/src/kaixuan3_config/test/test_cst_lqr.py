"""Robot-side LQI mathematics and bounded-output tests; no hardware transport."""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.linalg import expm
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/"scripts"))
from cst_cartesian_math import JOINTS, RobotModel, joint_reference, slew_torque
from cst_cartesian_lqr import (InputDynamics, MultiInputDynamics, LQRTorqueFeedback, design_schedule, linearize,
                              state_cost, torque_change_system)


class LQRTests(unittest.TestCase):
    def test_multiple_actuator_queues_match_the_coupled_discrete_plant(self):
        settings=[dict(joint='left_joint3',delay_steps=2,time_constant_s=.0044),
                  dict(joint='left_joint4',delay_steps=3,time_constant_s=.0021)]
        dynamics=MultiInputDynamics({'channels':settings})
        a,b=linearize(self.model,self.q,self.zero,self.zero)
        ad,bd,_=dynamics.discretize(a,b,np.eye(21),np.eye(7),.001)
        state=np.zeros(28);command=np.array([0,0,1.,-.5,0,0,0])
        for k in range(30):
            state=ad@state+bd@command;dynamics.advance(command,.001)
            np.testing.assert_allclose(state[21:],dynamics.state,atol=1e-13)
            if k<2:self.assertEqual(state[21],0.)
            if k<3:self.assertEqual(state[24],0.)
        np.testing.assert_array_equal(dynamics.nominal(command),[1,1,1,-.5,-.5,-.5,-.5])
        single=InputDynamics(settings[0]);multi=MultiInputDynamics({'channels':[settings[0]]})
        for left,right in zip(single.discretize(a,b,np.eye(21),np.eye(7),.001),multi.discretize(a,b,np.eye(21),np.eye(7),.001)):
            np.testing.assert_allclose(left,right,atol=1e-14)
        with self.assertRaises(ValueError):MultiInputDynamics({'channels':[settings[0],settings[0]]})

    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load((PACKAGE/"config/cst_cartesian_lqr.yaml").read_text())
        cls.model = RobotModel((PACKAGE.parent/"xiaogong_kaixuan/urdf/xiaogong-kaixuan.urdf").read_text(),
                               cls.config, np.zeros(7))
        cls.q = np.deg2rad(cls.config["target_joints_deg"])
        cls.zero = np.zeros(7)
        cls.schedule = design_schedule(cls.model, lambda t: (cls.q, cls.zero, cls.zero), 0., cls.config)
        raw = yaml.safe_load((PACKAGE.parent/"kaixuan_can_hardware/config/cartesian_torque_limits.yaml").read_text())["joints"]
        cls.limits = {key: np.array([raw[j][key] for j in JOINTS])
                      for key in ("effort_limit_nm", "effort_slew_nm_s")}

    def control(self):
        control = LQRTorqueFeedback(self.model, self.config, self.limits)
        control.schedules = {"test": self.schedule, "next": self.schedule}
        control.select("test", 0.)
        return control

    def test_release_slew_preserves_build_rate_and_crossing_time(self):
        previous = np.array([.5, -.5, .015, -.015, 0., .5, -.5])
        desired = np.array([.1, -.1, -.5, .5, .5, .7, -.7])
        result = slew_torque(previous, desired, .001, np.full(7, 7.5), np.full(7, 30.))
        np.testing.assert_allclose(result, [.47, -.47, -.00375, .00375, .0075, .5075, -.5075])
        ordinary = slew_torque(previous, desired, .001, np.full(7, 7.5), np.full(7, 7.5))
        np.testing.assert_allclose(ordinary, previous+np.clip(desired-previous, -.0075, .0075))

    def test_input_model_queue_is_causal_and_matches_augmented_plant(self):
        settings = dict(joint="left_joint4", delay_steps=3, time_constant_s=.0034)
        dynamics = InputDynamics(settings)
        a, b = linearize(self.model, self.q, self.zero, self.zero)
        ad, bd, _ = dynamics.discretize(a, b, np.eye(21), np.eye(7), .002)
        state, command = np.zeros(25), self.zero.copy()
        command[3] = 1.
        for k in range(12):
            state = ad@state+bd@command
            dynamics.advance(command, .002)
            np.testing.assert_allclose(dynamics.state, state[21:], atol=1e-13)
            if k < 3:
                np.testing.assert_allclose(state[:22], 0., atol=1e-14)
        self.assertGreater(state[21], .9)

    def test_delay_aware_lqi_stabilizes_delayed_plant_and_rejects_bias(self):
        config = copy.deepcopy(self.config)
        config["control_hz"] = 500.
        config["lqr"]["input_dynamics"] = dict(joint="left_joint4", delay_steps=3, time_constant_s=.0034)
        schedule = design_schedule(self.model, lambda t: (self.q, self.zero, self.zero), 0., config)
        dynamics = InputDynamics(config["lqr"]["input_dynamics"])
        a, b = linearize(self.model, self.q, self.zero, self.zero)
        ad, bd, _ = dynamics.discretize(a, b, np.eye(21), np.eye(7), .002)
        closed = ad-bd@schedule.gains[0]
        self.assertLess(max(abs(np.linalg.eigvals(closed))), 1.)
        disturbance = self.zero.copy(); disturbance[3] = .5
        equilibrium = np.linalg.solve(np.eye(25)-closed, bd@disturbance)
        np.testing.assert_allclose(equilibrium[:14], 0., atol=1e-10)
        np.testing.assert_allclose(-schedule.gains[0]@equilibrium, -disturbance, atol=1e-8)
        control = LQRTorqueFeedback(self.model, config, self.limits)
        control.schedules = {"test": schedule}; control.select("test", 0.)
        command, _, _ = control.step(self.q, self.zero, (self.q, self.zero, self.zero), .002, 0.)
        np.testing.assert_array_equal(command, 0.)

    def test_reference_friction_is_bounded_odd_and_zero_when_reference_stops(self):
        config = copy.deepcopy(self.config)
        config["reference_friction_nm"] = [0, 0, 0, 2.5, 0, 0, 0]
        control = LQRTorqueFeedback(self.model, config, self.limits)
        control.schedules = {"test": self.schedule}; control.select("test", 0.)
        for speed in [-1., 0., 1.]:
            velocity = self.zero.copy(); velocity[3] = speed
            control.step(self.q, velocity, (self.q, velocity, self.zero), .005)
            self.assertLessEqual(abs(control.friction_feedforward[3]), 2.5)
            self.assertEqual(np.sign(control.friction_feedforward[3]), np.sign(speed))
            np.testing.assert_allclose(control.feedforward-control.friction_feedforward,
                                       self.model.inverse_dynamics(self.q, velocity, self.zero))

    def test_fixed_torque_linearization_matches_nonlinear_finite_difference(self):
        velocity, acceleration = np.arange(7)*.03, np.arange(7)*.07
        torque = self.model.inverse_dynamics(self.q, velocity, acceleration)
        a, b = linearize(self.model, self.q, velocity, acceleration)
        x = np.r_[self.q, velocity]
        def dynamics(state, effort):
            return np.r_[state[7:], np.linalg.solve(self.model.mass(state[:7]),
                         effort-self.model.bias(state[:7], state[7:]))]
        epsilon = 1e-6
        numeric_a = np.column_stack([(dynamics(x+np.eye(14)[i]*epsilon, torque)
                                     -dynamics(x-np.eye(14)[i]*epsilon, torque))/(2*epsilon)
                                    for i in range(14)])
        numeric_b = np.column_stack([(dynamics(x, torque+np.eye(7)[i]*epsilon)
                                     -dynamics(x, torque-np.eye(7)[i]*epsilon))/(2*epsilon)
                                    for i in range(7)])
        np.testing.assert_allclose(a[:14, :14], numeric_a, rtol=2e-5, atol=2e-4)
        np.testing.assert_allclose(b[:14], numeric_b, rtol=1e-6, atol=2e-5)

    def test_riccati_residual_stability_and_coupled_feedback(self):
        d = self.schedule.diagnostics
        self.assertLess(d["terminal_dare_relative_residual"], 1e-9)
        self.assertLess(d["max_sampled_local_spectral_radius"], 1.)
        position = self.schedule.gains[0, :, :7]
        self.assertGreater(np.max(np.abs(position-np.diag(np.diag(position)))), 1.)

    def test_higher_effort_penalty_changes_optimal_gain(self):
        config = copy.deepcopy(self.config)
        config["lqr"]["effort_weight"] *= 4
        other = design_schedule(self.model, lambda t: (self.q, self.zero, self.zero), 0., config)
        self.assertLess(np.linalg.norm(other.gains[0]), np.linalg.norm(self.schedule.gains[0]))

    def test_nominal_torque_has_gravity_and_armature_exactly_once(self):
        control = self.control()
        acceleration = np.arange(7)*.01
        expected = self.model.mass(self.q)@acceleration+self.model.bias(self.q, self.zero)
        control.previous = expected.copy()
        command, _, integral = control.step(self.q, self.zero, (self.q, self.zero, acceleration), .005)
        np.testing.assert_allclose(command, expected, atol=1e-11)
        np.testing.assert_allclose(integral, 0., atol=1e-12)

    def test_integral_rejects_constant_input_disturbance_on_design_plant(self):
        a, b = linearize(self.model, self.q, self.zero, self.zero)
        zoh = np.zeros((28, 28)); zoh[:21, :21], zoh[:21, 21:] = a, b
        transition = expm(zoh*.005)
        ad, bd = transition[:21, :21], transition[:21, 21:]
        disturbance = np.array([.4, -.3, .2, -.2, .01, -.01, .01])
        state = np.zeros(21)
        for _ in range(8000):
            state = ad@state+bd@(-self.schedule.gains[0]@state+disturbance)
        self.assertLess(np.max(np.abs(state[:7])), 1e-5)
        np.testing.assert_allclose(-self.schedule.gains[0]@state, -disturbance, atol=1e-4)

    def test_output_integral_bounds_ramp_and_bumpless_phase_change(self):
        control = self.control()
        ref = (self.q, self.zero, self.zero)
        command, _, integral = control.step(self.q-.1, self.zero, ref, .005, scale=0.)
        np.testing.assert_array_equal(command, 0.)
        np.testing.assert_array_equal(integral, 0.)
        last = np.zeros(7)
        for _ in range(400):
            command, _, integral = control.step(self.q-.2, self.zero, ref, .005)
            self.assertTrue(np.all(np.abs(command) <= self.limits["effort_limit_nm"]+1e-12))
            self.assertTrue(np.all(np.abs(command-last) <= self.limits["effort_slew_nm_s"]*.005+1e-12))
            self.assertTrue(np.all(np.abs(integral) <= np.array(self.config["integral_limit_nm"])+1e-12))
            last = command
        old = control.integral.copy()
        control.select("next", 0.)
        np.testing.assert_allclose(-control.gain[:, 14:]@control.z, old, atol=1e-12)

    def test_moving_reference_produces_time_varying_200hz_gains(self):
        end = self.q+np.deg2rad([2, -1, 2, 2, 0, 0, 0])
        schedule = design_schedule(self.model, lambda t: joint_reference(self.q, end, t, .3), .3, self.config)
        self.assertAlmostEqual(schedule.period, .005)
        self.assertGreater(np.linalg.norm(schedule.gains[0]-schedule.gains[-1]), .01)
        np.testing.assert_array_equal(schedule.sample(20.)[0], schedule.gains[-1])

    def rate_design(self):
        config = copy.deepcopy(self.config)
        config["lqr"]["torque_change_time_s"] = .02
        design = design_schedule(self.model, lambda t: (self.q, self.zero, self.zero), 0., config)
        a, b = linearize(self.model, self.q, self.zero, self.zero)
        zoh = np.zeros((28, 28)); zoh[:21, :21], zoh[:21, 21:] = a, b
        transition = expm(zoh*.005)
        ad, bd = transition[:21, :21], transition[:21, 21:]
        r = np.diag(config["lqr"]["effort_weight"]/np.asarray(config["lqr"]["torque_scale_nm"])**2)*.005
        aa, bb, _, _ = torque_change_system(ad, bd, state_cost(self.model, self.q, config["lqr"])*.005,
                                           r, .005, .02)
        return config, design, ad, bd, aa, bb

    def test_increment_input_matches_held_torque_plant_and_rejects_bias(self):
        _, design, ad, bd, aa, bb = self.rate_design()
        state = np.arange(28)*.0001
        increment = np.arange(7)*.001
        next_state = aa@state+bb@increment
        np.testing.assert_allclose(next_state[:21], ad@state[:21]+bd@(state[21:]+increment), atol=1e-14)
        np.testing.assert_allclose(next_state[21:], state[21:]+increment, atol=1e-14)
        disturbance = np.array([.4, -.3, .2, -.2, .01, -.01, .01])
        closed = aa-bb@design.gains[0]
        equilibrium = np.linalg.solve(np.eye(28)-closed, np.r_[bd@disturbance, np.zeros(7)])
        np.testing.assert_allclose(equilibrium[:14], 0., atol=1e-9)
        np.testing.assert_allclose(equilibrium[21:], -disturbance, atol=1e-9)

    def test_increment_cost_attenuates_17hz_feedback_torque_response(self):
        _, rate, ad, bd, aa, bb = self.rate_design()
        z = np.exp(2j*np.pi*17*.005)
        k = self.schedule.gains[0]
        ordinary = -k@np.linalg.solve(z*np.eye(21)-(ad-bd@k), -bd@k[:, 10])-k[:, 10]
        k = rate.gains[0]
        output = np.hstack((np.zeros((7, 21)), np.eye(7)))-k
        shaped = output@np.linalg.solve(z*np.eye(28)-(aa-bb@k), -bb@k[:, 10])-k[:, 10]
        self.assertLess(np.linalg.norm(shaped), .8*np.linalg.norm(ordinary))

    def test_increment_controller_preserves_zero_ramp_and_physical_limits(self):
        config, schedule, *_ = self.rate_design()
        control = LQRTorqueFeedback(self.model, config, self.limits)
        control.schedules = {"test": schedule}; control.select("test", 0.)
        ref = (self.q, self.zero, self.zero)
        command, _, _ = control.step(self.q, self.zero, ref, .005, scale=0.)
        np.testing.assert_array_equal(command, 0.)
        last = command
        for _ in range(400):
            command, raw, integral = control.step(self.q-.1, self.zero, ref, .005)
            self.assertTrue(np.isfinite(raw).all())
            self.assertTrue(np.all(np.abs(command) <= self.limits["effort_limit_nm"]+1e-12))
            self.assertTrue(np.all(np.abs(command-last) <= self.limits["effort_slew_nm_s"]*.005+1e-12))
            self.assertTrue(np.all(np.abs(integral) <= np.array(config["integral_limit_nm"])+1e-12))
            last = command


if __name__ == "__main__":
    unittest.main()
