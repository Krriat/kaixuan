"""Coupled TV-LQI design and bounded torque feedback; no ROS or EtherCAT access."""
from dataclasses import dataclass
import time

import numpy as np
from scipy.linalg import expm

from cst_cartesian_math import JOINTS, joint_reference, pin, vector, slew_torque
from kaixuan_control_lab.lqr import design_tvlqr


def linearize(model, q, dq, ddq):
    """Fixed-input derivatives of M(q)^-1 (tau-h(q,v)), including armature once."""
    dtau_dq, dtau_dv, _ = pin.computeRNEADerivatives(model.model, model.data, q, dq, ddq)
    # mass() overwrites Pinocchio data, so copy derivatives first.
    derivatives = np.hstack((dtau_dq.copy(), dtau_dv.copy()))
    mass = model.mass(q)
    a, b = np.zeros((21, 21)), np.zeros((21, 7))
    a[:7, 7:14] = np.eye(7)
    a[7:14, :14] = -np.linalg.solve(mass, derivatives)
    a[14:, :7] = np.eye(7)  # z_dot = q-q_reference, in rad.s
    b[7:14] = np.linalg.solve(mass, np.eye(7))
    return a, b


def state_cost(model, q, settings):
    jac = model.kinematics(q)[2]
    position = vector(settings, "joint_position_scale_rad", True)
    velocity = vector(settings, "joint_velocity_scale_rad_s", True)
    integral = vector(settings, "integrated_error_scale_rad_s", True)
    task_scale = np.array([settings["ee_position_scale_m"]]*3
                          + [settings["ee_orientation_scale_rad"]]*3)
    if not np.isfinite(task_scale).all() or np.any(task_scale <= 0):
        raise ValueError("LQR task scales must be finite and positive")
    cost = np.diag(np.r_[1/position**2, 1/velocity**2, 1/integral**2])
    cost[:7, :7] += jac.T @ np.diag(1/task_scale**2) @ jac
    return cost


def torque_change_system(ad, bd, q, torque_cost, period, time_scale):
    """Augment with previous feedback torque; optimize its increment.

    x = [position error, velocity error, integral error, previous torque error].
    Actual feedback torque u_k = u_(k-1) + du_k. Its previous value is penalized
    in Q, and its rate du_k/dt is penalized in R. A terminal DARE handles the
    boundary term between previous/current torque costs.
    """
    a, b, cost = np.zeros((28, 28)), np.zeros((28, 7)), np.zeros((28, 28))
    a[:21, :21], a[:21, 21:], a[21:, 21:] = ad, bd, np.eye(7)
    b[:21], b[21:] = bd, np.eye(7)
    cost[:21, :21], cost[21:, 21:] = q, torque_cost
    return a, b, cost, torque_cost*(time_scale/period)**2


class InputDynamics:
    """One identified input channel: first-order lag plus integer sample delay.

    Hidden states are absolute applied torque and the command delay queue at
    runtime; subtract nominal torque before applying the LQR error gain.
    """
    def __init__(self, settings):
        self.joint = JOINTS.index(settings["joint"])
        self.delay = settings["delay_steps"]
        self.time_constant = float(settings["time_constant_s"])
        self.weight = float(settings.get("state_effort_weight", .01))
        if (not isinstance(self.delay, int) or not 0 <= self.delay <= 10
                or not np.isfinite([self.time_constant, self.weight]).all()
                or not 0. < self.time_constant <= .05 or self.weight <= 0.):
            raise ValueError("Invalid identified input dynamics")
        self.state = np.zeros(1+self.delay)

    def discretize(self, a, b, q, r, period):
        continuous = np.zeros((29, 29))
        continuous[:21, :21] = a
        continuous[:21, 22:] = b
        continuous[:21, 22+self.joint] = 0.
        continuous[:21, 21] = b[:, self.joint]
        continuous[21, 21] = -1./self.time_constant
        continuous[21, 22+self.joint] = 1./self.time_constant
        zoh = expm(continuous*period)
        size = 22+self.delay
        ad, bd, cost = np.zeros((size, size)), np.zeros((size, 7)), np.zeros((size, size))
        ad[:22, :22], bd[:22] = zoh[:22, :22], zoh[:22, 22:]
        cost[:21, :21] = q
        cost[21:, 21:] = np.eye(1+self.delay)*self.weight*r[self.joint, self.joint]
        if self.delay:
            ad[:22, 22] = bd[:22, self.joint]
            bd[:22, self.joint] = 0.
            for index in range(22, size-1):
                ad[index, index+1] = 1.
            bd[-1, self.joint] = 1.
        return ad, bd, cost

    def advance(self, previous_command, dt):
        delayed = self.state[1] if self.delay else previous_command[self.joint]
        rho = np.exp(-dt/self.time_constant)
        self.state[0] = rho*self.state[0] + (1.-rho)*delayed
        if self.delay:
            self.state[1:-1] = self.state[2:]
            self.state[-1] = previous_command[self.joint]

    def nominal(self, feedforward):
        return np.full(len(self.state), feedforward[self.joint])


class MultiInputDynamics:
    """Independent actuator lag/delay states coupled through the full arm model."""
    def __init__(self, settings):
        self.channels = [InputDynamics(s) for s in settings["channels"]]
        if not self.channels or len({c.joint for c in self.channels}) != len(self.channels):
            raise ValueError("Input channels must select distinct joints")
        self.offsets = np.cumsum([0]+[len(c.state) for c in self.channels])
        self.state = np.zeros(self.offsets[-1])
        if len(self.state) > 11:
            raise ValueError("Native servo supports at most eleven actuator states")
        for c, start, end in zip(self.channels, self.offsets[:-1], self.offsets[1:]):
            c.state = self.state[start:end]

    def discretize(self, a, b, q, r, period):
        count = len(self.channels)
        physical = 21+count
        continuous = np.zeros((physical+7, physical+7))
        continuous[:21, :21], continuous[:21, physical:] = a, b
        for k, c in enumerate(self.channels):
            continuous[:21, physical+c.joint] = 0.
            continuous[:21, 21+k] = b[:, c.joint]
            continuous[21+k, 21+k] = -1./c.time_constant
            continuous[21+k, physical+c.joint] = 1./c.time_constant
        zoh = expm(continuous*period)
        size = 21+len(self.state)
        ad, bd, cost = np.zeros((size, size)), np.zeros((size, 7)), np.zeros((size, size))
        mapped = np.r_[np.arange(21), 21+self.offsets[:-1]]
        ad[np.ix_(mapped, mapped)] = zoh[:physical, :physical]
        bd[mapped] = zoh[:physical, physical:]
        cost[:21, :21] = q
        for c, offset in zip(self.channels, self.offsets[:-1]):
            start = 21+offset
            cost[start:start+1+c.delay, start:start+1+c.delay] = np.eye(1+c.delay)*c.weight*r[c.joint, c.joint]
            if c.delay:
                ad[mapped, start+1] = bd[mapped, c.joint]
                bd[mapped, c.joint] = 0.
                for k in range(start+1, start+c.delay):
                    ad[k, k+1] = 1.
                bd[start+c.delay, c.joint] = 1.
        return ad, bd, cost

    def advance(self, previous_command, dt):
        for c in self.channels:
            c.advance(previous_command, dt)

    def nominal(self, feedforward):
        return np.concatenate([c.nominal(feedforward) for c in self.channels])


def input_dynamics(settings):
    return MultiInputDynamics(settings) if "channels" in settings else InputDynamics(settings)


@dataclass
class Schedule:
    period: float
    gains: np.ndarray
    inverse_integral_gains: np.ndarray
    diagnostics: dict

    def sample(self, elapsed):
        index = min(max(int(elapsed/self.period), 0), len(self.gains)-1)
        return self.gains[index], self.inverse_integral_gains[index]


def design_schedule(model, reference, duration, config):
    """Riccati recursion at the feedback runtime rate, with a coarser model mesh.

    The plant is piecewise constant between linearization knots. The backward
    recursion and gain playback use the nominal feedback period. With the
    native backend, references/gains can be streamed at a lower rate.
    """
    started = time.monotonic()
    settings = config["lqr"]
    runtime_hz = float(settings.get("runtime_hz", config["control_hz"]))
    if runtime_hz != config["control_hz"] and config.get("runtime_backend") != "ros2_control":
        raise ValueError("A separate runtime_hz requires the ros2_control backend")
    if not np.isfinite(runtime_hz) or not 100 <= runtime_hz <= 1000:
        raise ValueError("Invalid LQR runtime frequency")
    period, mesh = 1/runtime_hz, float(settings["linearization_period_s"])
    if not np.isfinite(mesh) or mesh < period or mesh > .1:
        raise ValueError("LQR linearization period must be between control period and 0.1 s")
    effort_weight = float(settings["effort_weight"])
    if not np.isfinite(effort_weight) or effort_weight <= 0:
        raise ValueError("LQR effort_weight must be positive")
    r = np.diag(effort_weight/vector(settings, "torque_scale_nm", True)**2)*period
    rate_time = float(settings.get("torque_change_time_s", 0.))
    if not np.isfinite(rate_time) or not 0. <= rate_time <= .1:
        raise ValueError("LQR torque_change_time_s must be between 0 and 0.1 s")
    increment_cost = None
    input_model = input_dynamics(settings["input_dynamics"]) if "input_dynamics" in settings else None
    if input_model is not None and rate_time:
        raise ValueError("Choose input dynamics or torque-change cost, not both")
    knots = np.linspace(0., duration, max(1, int(np.ceil(duration/mesh)))+1) if duration else np.array([0.])
    ad, bd, costs = [], [], []
    for t in knots:
        q, dq, ddq = reference(t)
        a, b = linearize(model, q, dq, ddq)
        qk = state_cost(model, q, settings)*period
        if input_model is not None:
            ak, bk, qk = input_model.discretize(a, b, qk, r, period)
        else:
            zoh = np.zeros((28, 28))
            zoh[:21, :21], zoh[:21, 21:] = a, b
            discrete = expm(zoh*period)
            ak, bk = discrete[:21, :21], discrete[:21, 21:]
        if rate_time:
            ak, bk, qk, increment_cost = torque_change_system(ak, bk, qk, r, period, rate_time)
        ad.append(ak)
        bd.append(bk)
        costs.append(qk)
    times = np.arange(max(1, int(np.ceil(duration/period)))+1)*period if duration else np.array([0.])
    indices = np.maximum(np.searchsorted(knots, np.minimum(times, duration), side="right")-1, 0)
    ad, bd, costs = [np.asarray(values)[indices] for values in (ad, bd, costs)]
    design = design_tvlqr(ad, bd, costs, increment_cost if rate_time else r)
    gains = design.gains
    inverse = np.linalg.inv(gains[:, :, 14:21])
    # Evaluate frozen local poles at representative knots, in addition to the
    # terminal DARE/Bellman checks. This is not a nonlinear stability proof.
    selected = np.unique(np.r_[np.searchsorted(times, knots), len(times)-1]).clip(0, len(times)-1)
    radius = max(float(np.max(np.abs(np.linalg.eigvals(ad[k]-bd[k]@gains[k])))) for k in selected)
    if radius >= 1. or not np.isfinite(inverse).all():
        raise RuntimeError(f"LQI local design failed: spectral radius {radius}")
    diagnostics = dict(design.diagnostics, nominal_control_period_s=period,
                       linearization_period_s=mesh, linearization_knots=len(knots),
                       torque_change_time_s=rate_time, state_dimension=gains.shape[-1],
                       optimized_input="feedback torque increment" if rate_time else "feedback torque",
                       max_sampled_local_spectral_radius=radius,
                       integral_gain_condition_max=float(max(np.linalg.cond(gains[k, :, 14:21]) for k in selected)),
                       position_gain_diagonal_start=np.diag(gains[0, :, :7]).tolist(),
                       velocity_gain_diagonal_start=np.diag(gains[0, :, 7:14]).tolist(),
                       integral_gain_diagonal_start=np.diag(gains[0, :, 14:21]).tolist(),
                       synthesis_wall_s=time.monotonic()-started)
    if input_model is not None:
        raw = settings["input_dynamics"]
        channels = raw.get("channels", [raw])
        diagnostics["input_dynamics"] = {"channels": [dict(c, delay_s=c["delay_steps"]*period) for c in channels]}
    return Schedule(period, gains.copy(), inverse, diagnostics)


class LQRTorqueFeedback:
    """TV-LQI; optional previous-input state penalizes feedback torque changes.

    Default: tau = RNEA(qr,vr,ar) - K(t)[q-qr;v-vr;integral(q-qr)].
    With torque_change_time_s>0, optimize torque increments using 28 states.

    Integral torque is bounded and conditionally integrated during saturation.
    At phase changes its physical torque contribution is preserved (bumpless
    transfer), rather than reusing integral coordinates with a different Kz.
    Amplitude/slew clipping is a safety wrapper, not an LQR constraint solver.
    """
    def __init__(self, model, config, limits):
        self.model, self.config = model, config
        self.limit, self.slew = limits["effort_limit_nm"], limits["effort_slew_nm_s"]
        self.release_slew = limits.get("effort_release_slew_nm_s", self.slew)
        self.integral_limit = vector(config, "integral_limit_nm", True)
        self.previous, self.integral, self.z = np.zeros(7), np.zeros(7), np.zeros(7)
        self.schedules, self.phase = {}, None
        self.gain = self.inverse_integral_gain = None
        self.previous_feedforward = None
        settings = config["lqr"].get("input_dynamics")
        self.input_model = input_dynamics(settings) if settings is not None else None
        self.friction = np.asarray(config.get("reference_friction_nm", [0.]*7), dtype=float)
        self.friction_speed = float(config.get("reference_friction_transition_rad_s", .01))
        if (self.friction.shape != (7,) or not np.isfinite(self.friction).all()
                or np.any(self.friction < 0.) or np.any(self.friction > self.limit)
                or not np.isfinite(self.friction_speed) or self.friction_speed <= 0.):
            raise ValueError("Invalid bounded reference friction feedforward")

    def prepare(self, initial, reference):
        target = np.deg2rad(vector(self.config, "target_joints_deg"))
        zero = np.zeros(7)
        duration = self.config["approach_s"]
        line_duration = float(reference.times[-1])
        definitions = {
            "initial": (lambda t: (initial, zero, zero), 0.),
            "target": (lambda t: (target, zero, zero), 0.),
            "final": (lambda t: (reference.q[-1], zero, zero), 0.),
            "approach": (lambda t: joint_reference(initial, target, t, duration), duration),
            "line": (reference.sample, line_duration),
            "return_line": (lambda t: self.reverse_reference(reference, t), line_duration),
            "return_initial": (lambda t: joint_reference(target, initial, t, duration), duration),
        }
        for name, (function, seconds) in definitions.items():
            print(f"Designing TV-LQI {name}: {seconds:.1f}s (drives disabled)", flush=True)
            self.schedules[name] = design_schedule(self.model, function, seconds, self.config)
        aliases = {"enable_zero": "initial", "ramp": "initial", "settle_initial": "initial",
                   "settle_target": "target", "settle_final": "final", "final_hold": "final",
                   "settle_return_target": "target", "settle_return_initial": "initial",
                   "ramp_down_at_rest": "initial", "hold": "final", "commission_hold": "initial"}
        for name, source in aliases.items():
            self.schedules[name] = self.schedules[source]

    @staticmethod
    def reverse_reference(reference, t):
        q, dq, ddq = reference.sample(reference.times[-1]-t)
        return q, -dq, ddq

    def select(self, phase, elapsed):
        gain, inverse = self.schedules[phase].sample(elapsed)
        if phase != self.phase:
            self.z = -inverse@self.integral
        self.gain, self.inverse_integral_gain, self.phase = gain, inverse, phase

    def step(self, q, dq, reference, dt, scale=1.):
        if self.gain is None:
            raise RuntimeError("Select a precomputed LQR phase before commanding")
        qr, dqr, ddqr = reference
        error = q-qr
        self.friction_feedforward = self.friction*np.tanh(dqr/self.friction_speed)
        self.feedforward = self.model.inverse_dynamics(qr, dqr, ddqr) + self.friction_feedforward
        self.state_feedback = -self.gain[:, :14]@np.r_[error, dq-dqr]
        self.memory_feedback = np.zeros(7)
        if self.input_model is not None:
            self.input_model.advance(self.previous, dt)
            nominal = self.input_model.nominal(scale*self.feedforward)
            self.memory_feedback = -scale*self.gain[:, 21:]@(self.input_model.state-nominal)
        elif self.gain.shape[1] == 28:
            if self.previous_feedforward is None:
                self.previous_feedforward = self.previous.copy()
            self.memory_feedback = (np.eye(7)-self.gain[:, 21:])@(self.previous-self.previous_feedforward)
        base = self.feedforward + self.state_feedback
        old_i = np.clip(-self.gain[:, 14:21]@self.z, -self.integral_limit, self.integral_limit)
        candidate_z = self.z + error*dt if scale >= 1. else self.z
        candidate_i = np.clip(-self.gain[:, 14:21]@candidate_z, -self.integral_limit, self.integral_limit)
        raw = scale*(base+candidate_i)+self.memory_feedback
        command = self.bound(raw, dt)
        # Ignore roundoff when an unsaturated slew calculation reconstructs raw.
        # Otherwise a ~1e-15 Nm difference can randomly freeze integration.
        outward = (raw-command)*np.sign(candidate_i-old_i) > 1e-10
        self.integral = np.where(outward, old_i, candidate_i)
        self.z = -self.inverse_integral_gain@self.integral
        raw = scale*(base+self.integral)+self.memory_feedback
        command = self.bound(raw, dt)
        self.previous = command.copy()
        self.previous_feedforward = scale*self.feedforward
        return command, raw, self.integral.copy()

    def bound(self, raw, dt):
        return slew_torque(self.previous, np.clip(raw, -self.limit, self.limit),
                           dt, self.slew, self.release_slew)
