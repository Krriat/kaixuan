"""Optional smooth cruise timing for the checked robot-side Cartesian line."""
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline

from kaixuan_control_lab.trajectory import (
    CartesianLineReference, build_reference, integer_steps, rotation_error, solve_ik,
)


def cruise_progress(time_s, move_s, ramp_s):
    """Unit displacement with quintic velocity ramps and a constant-speed middle.

    The area of each velocity ramp is half its duration, hence v=1/(T-a).
    Velocity, acceleration and jerk are continuous, including the end holds.
    """
    if not np.isfinite([move_s, ramp_s]).all() or not 0 < ramp_s <= move_s / 2:
        raise ValueError("cruise_ramp_s must be positive and at most half line_s")
    t = np.clip(np.asarray(time_s, dtype=float), 0., move_s)
    half_t = np.minimum(t, move_s-t)
    s = np.clip(half_t/ramp_s, 0., 1.)
    integral = s**4 * (2.5 - 3*s + s*s)
    half_progress = np.where(half_t < ramp_s, ramp_s*integral,
                             half_t-ramp_s/2) / (move_s-ramp_s)
    return np.where(t <= move_s/2, half_progress, 1.-half_progress)


@dataclass
class CruiseReference(CartesianLineReference):
    ramp_s: float

    def desired_position(self, time_s):
        progress = cruise_progress(np.asarray(time_s)-self.start_hold_s,
                                   self.move_s, self.ramp_s)
        return self.start_position + np.expand_dims(progress, -1)*self.displacement


def build_timed_reference(dynamics, q0, lower, upper, settings, physics_step_s):
    tr = settings["trajectory"]
    profile = tr.get("profile", "quintic")
    if profile == "quintic":
        return build_reference(dynamics, q0, lower, upper, settings, physics_step_s)
    if profile != "smooth_cruise":
        raise ValueError(f"Unknown Cartesian time profile: {profile}")
    ramp_s = float(tr["cruise_ramp_s"])
    cruise_progress(0., tr["move_s"], ramp_s)  # Validate before generating IK.
    dt = settings["control_period_s"]
    duration = tr["start_hold_s"] + tr["move_s"] + tr["end_hold_s"]
    count = integer_steps(duration, dt, "total duration")
    for key in ("start_hold_s", "move_s", "end_hold_s"):
        integer_steps(tr[key], dt, key)
    integer_steps(ramp_s, dt, "cruise_ramp_s")
    times = np.arange(count+1)*dt
    p0, r0, _ = dynamics.kinematics(q0)
    delta = np.asarray(tr["displacement_world_m"], dtype=float)
    positions = p0 + cruise_progress(times-tr["start_hold_s"], tr["move_s"], ramp_s)[:, None]*delta
    qs, iterations, sigmas = [], [], []
    seed = np.asarray(q0, dtype=float)
    for position in positions:
        seed, it, sigma = solve_ik(dynamics, position, r0, seed, lower, upper,
                                  tolerance=tr["ik_tolerance"], max_iterations=tr["max_ik_iterations"])
        qs.append(seed.copy())
        iterations.append(it)
        sigmas.append(sigma)
    q = np.array(qs)
    spline = CubicSpline(times, q, axis=0, bc_type=((1, np.zeros(len(q0))), (1, np.zeros(len(q0)))))
    dq, ddq = spline(times, 1), spline(times, 2)
    tau = np.array([dynamics.mass(a)@c + dynamics.bias(a, b) for a, b, c in zip(q, dq, ddq)])
    reference = CruiseReference(times, positions, r0, q, dq, ddq, tau, spline, p0,
                               delta, tr["start_hold_s"], tr["move_s"], {}, ramp_s)
    dense_times = np.arange(integer_steps(duration, physics_step_s, "duration")+1)*physics_step_s
    dense_q, dense_dq, _ = reference.sample(dense_times)
    position_errors, orientation_errors = [], []
    for t, angles in zip(dense_times, dense_q):
        p, r, _ = dynamics.kinematics(angles)
        position_errors.append(np.linalg.norm(p-reference.desired_position(t)))
        orientation_errors.append(np.linalg.norm(rotation_error(r0, r)))
    reference.diagnostics = {
        "time_profile": profile, "cruise_ramp_s": ramp_s,
        "duration_s": duration, "line_length_m": float(np.linalg.norm(delta)),
        "max_position_error_m": float(max(position_errors)),
        "max_orientation_error_rad": float(max(orientation_errors)),
        "min_6d_jacobian_singular_value_mixed_units": float(min(sigmas)),
        "max_ik_iterations": int(max(iterations)),
        "joint_excursion_deg": np.rad2deg(np.ptp(dense_q, axis=0)).tolist(),
        "peak_reference_velocity_rad_s": np.max(np.abs(dense_dq), axis=0).tolist(),
        "peak_reference_torque_nm": np.max(np.abs(tau), axis=0).tolist(),
    }
    if (max(position_errors) > settings["acceptance"]["reference_position_m"]
            or max(orientation_errors) > settings["acceptance"]["reference_orientation_rad"]):
        raise RuntimeError("Interpolated Cartesian cruise reference failed its geometry check")
    if np.any(dense_q <= lower) or np.any(dense_q >= upper):
        raise RuntimeError("Interpolated cruise reference exceeds joint limits")
    return reference
