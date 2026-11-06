"""A fixed-orientation Cartesian line, continuous IK, and smooth joint references."""
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation


def integer_steps(period_s, physics_step_s, label):
    if not np.isfinite([period_s, physics_step_s]).all() or min(period_s, physics_step_s) <= 0:
        raise ValueError(f"{label} and physics step must be finite and positive")
    ratio = period_s / physics_step_s
    result = round(ratio)
    if result < 1 or not np.isclose(ratio, result, rtol=0, atol=1e-9):
        raise ValueError(f"{label} must be an integer multiple of the physics step")
    return result


def rotation_error(target, actual):
    """World-axis rotation vector, consistent with LOCAL_WORLD_ALIGNED angular J."""
    return Rotation.from_matrix(target @ actual.T).as_rotvec()


def solve_ik(dynamics, position, rotation, seed, lower, upper, *, tolerance=1e-10,
             max_iterations=40):
    q = np.asarray(seed, dtype=float).copy()
    for iteration in range(max_iterations):
        actual, orient, jacobian = dynamics.kinematics(q)
        error = np.r_[position - actual, rotation_error(rotation, orient)]
        if np.max(np.abs(error)) <= tolerance:
            if np.any(q <= lower) or np.any(q >= upper):
                raise RuntimeError("IK solution violates a joint limit")
            return q, iteration, float(np.linalg.svd(jacobian, compute_uv=False)[-1])
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-10 * np.eye(6), error)
        delta *= min(1., .1 / max(1e-15, np.max(np.abs(delta))))
        q += delta
        if np.any(q <= lower) or np.any(q >= upper):
            raise RuntimeError("IK step left the configured joint range; choose another line")
    raise RuntimeError("IK did not converge; no reference accepted")


@dataclass
class CartesianLineReference:
    times: np.ndarray
    positions: np.ndarray
    orientation: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    ddq: np.ndarray
    tau: np.ndarray
    spline: CubicSpline
    start_position: np.ndarray
    displacement: np.ndarray
    start_hold_s: float
    move_s: float
    diagnostics: dict

    def sample(self, time_s):
        time_s = np.clip(time_s, self.times[0], self.times[-1])
        return self.spline(time_s), self.spline(time_s, 1), self.spline(time_s, 2)

    def desired_position(self, time_s):
        s = np.clip((np.asarray(time_s) - self.start_hold_s) / self.move_s, 0., 1.)
        progress = s**3 * (10 - 15*s + 6*s*s)
        return self.start_position + np.expand_dims(progress, -1) * self.displacement


def build_reference(dynamics, q0, lower, upper, settings, physics_step_s):
    tr = settings["trajectory"]
    dt = settings["control_period_s"]
    duration = tr["start_hold_s"] + tr["move_s"] + tr["end_hold_s"]
    count = integer_steps(duration, dt, "total duration")
    for key in ("start_hold_s", "move_s", "end_hold_s"):
        integer_steps(tr[key], dt, key)
    times = np.arange(count + 1) * dt
    p0, r0, _ = dynamics.kinematics(q0)
    delta = np.asarray(tr["displacement_world_m"], dtype=float)
    s = np.clip((times - tr["start_hold_s"]) / tr["move_s"], 0., 1.)
    positions = p0 + (s**3 * (10 - 15*s + 6*s*s))[:, None] * delta
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
    tau = np.array([dynamics.mass(a) @ c + dynamics.bias(a, b) for a, b, c in zip(q, dq, ddq)])
    reference = CartesianLineReference(times, positions, r0, q, dq, ddq, tau, spline, p0,
        delta, tr["start_hold_s"], tr["move_s"], {})
    dense_times = np.arange(integer_steps(duration, physics_step_s, "duration") + 1) * physics_step_s
    dense_q = reference.sample(dense_times)[0]
    position_errors, orientation_errors = [], []
    for t, angles in zip(dense_times, dense_q):
        p, r, _ = dynamics.kinematics(angles)
        position_errors.append(np.linalg.norm(p - reference.desired_position(t)))
        orientation_errors.append(np.linalg.norm(rotation_error(r0, r)))
    reference.diagnostics = {
        "duration_s": duration, "line_length_m": float(np.linalg.norm(delta)),
        "max_position_error_m": float(max(position_errors)),
        "max_orientation_error_rad": float(max(orientation_errors)),
        "min_6d_jacobian_singular_value_mixed_units": float(min(sigmas)),
        "max_ik_iterations": int(max(iterations)),
        "joint_excursion_deg": np.rad2deg(np.ptp(dense_q, axis=0)).tolist(),
        "peak_reference_velocity_rad_s": np.max(np.abs(reference.sample(dense_times)[1]), axis=0).tolist(),
        "peak_reference_torque_nm": np.max(np.abs(tau), axis=0).tolist(),
    }
    if (max(position_errors) > settings["acceptance"]["reference_position_m"]
            or max(orientation_errors) > settings["acceptance"]["reference_orientation_rad"]):
        raise RuntimeError("Interpolated Cartesian reference failed its geometry check")
    if np.any(dense_q <= lower) or np.any(dense_q >= upper):
        raise RuntimeError("Interpolated reference exceeds joint limits")
    return reference
