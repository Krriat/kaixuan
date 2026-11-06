"""Multi-joint linearization and finite-horizon, time-varying LQR (total Nm)."""
from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm, solve_discrete_are


def forward_dynamics(dynamics, x, torque):
    n = len(torque)
    q, dq = x[:n], x[n:]
    return np.r_[dq, np.linalg.solve(dynamics.mass(q), torque - dynamics.bias(q, dq))]


def linearize_dynamics(dynamics, q, dq, torque, dt, *, epsilon=1e-5):
    """Differentiate the coupled nonlinear plant, then exact ZOH for its local LTI model.

    No disturbance, torque clipping, delay, or motor filter is part of this design
    model. Those are evaluated independently on the nonlinear physical plant.
    """
    n = len(q)
    if not np.isfinite([dt, epsilon]).all() or min(dt, epsilon) <= 0:
        raise ValueError("dt and difference epsilon must be finite and positive")
    x = np.r_[q, dq]
    a = np.zeros((2*n, 2*n))
    a[:n, n:] = np.eye(n)
    for j in range(2*n):
        delta = np.zeros(2*n)
        delta[j] = epsilon
        a[n:, j] = (forward_dynamics(dynamics, x + delta, torque)[n:]
                      - forward_dynamics(dynamics, x - delta, torque)[n:]) / (2*epsilon)
    b = np.vstack((np.zeros((n, n)), np.linalg.solve(dynamics.mass(q), np.eye(n))))
    augmented = np.zeros((3*n, 3*n))
    augmented[:2*n, :2*n], augmented[:2*n, 2*n:] = a, b
    transition = expm(augmented * dt)
    return a, b, transition[:2*n, :2*n], transition[:2*n, 2*n:]


def state_weights(dynamics, q, settings):
    w = settings["weights"]
    n = len(q)
    jac = dynamics.kinematics(q)[2]
    task_weights = np.diag([1/w["ee_position_scale_m"]**2]*3
                          + [1/w["ee_orientation_scale_rad"]**2]*3)
    qmat = np.zeros((2*n, 2*n))
    qmat[:n, :n] = jac.T @ task_weights @ jac + np.eye(n)/w["joint_position_scale_rad"]**2
    qmat[n:, n:] = np.eye(n)/w["joint_velocity_scale_rad_s"]**2
    return qmat * settings["control_period_s"]


@dataclass
class TVLQRDesign:
    ad: np.ndarray
    bd: np.ndarray
    q: np.ndarray
    r: np.ndarray
    p: np.ndarray
    gains: np.ndarray
    terminal_gain: np.ndarray
    diagnostics: dict


def design_tvlqr(ad, bd, q, r):
    ad, bd, q, r = [np.asarray(v, dtype=float) for v in (ad, bd, q, r)]
    if (ad.ndim != 3 or ad.shape[1] != ad.shape[2] or bd.ndim != 3
            or bd.shape[:2] != ad.shape[:2] or q.shape != ad.shape
            or r.shape != (bd.shape[2], bd.shape[2])
            or not all(np.isfinite(v).all() for v in (ad, bd, q, r))):
        raise ValueError("Inconsistent or nonfinite TVLQR matrices")
    if (not np.allclose(q, q.transpose(0, 2, 1)) or not np.allclose(r, r.T)
            or np.min(np.linalg.eigvalsh(q)) <= 0 or np.min(np.linalg.eigvalsh(r)) <= 0):
        raise ValueError("This experiment requires symmetric positive definite Q and R")
    terminal = solve_discrete_are(ad[-1], bd[-1], q[-1], r)
    terminal_gain = np.linalg.solve(r + bd[-1].T @ terminal @ bd[-1], bd[-1].T @ terminal @ ad[-1])
    terminal_residual = q[-1] + ad[-1].T @ terminal @ (ad[-1] - bd[-1] @ terminal_gain) - terminal
    terminal_poles = np.linalg.eigvals(ad[-1] - bd[-1] @ terminal_gain)
    p = np.empty((len(ad)+1, *ad.shape[1:]))
    gains = np.empty((len(ad), bd.shape[2], ad.shape[1]))
    p[-1] = .5*(terminal + terminal.T)
    residuals = []
    for k in reversed(range(len(ad))):
        gain = np.linalg.solve(r + bd[k].T @ p[k+1] @ bd[k], bd[k].T @ p[k+1] @ ad[k])
        closed = ad[k] - bd[k] @ gain
        # Bellman form avoids subtracting two large positive terms.
        current = q[k] + gain.T @ r @ gain + closed.T @ p[k+1] @ closed
        p[k] = .5*(current + current.T)
        gains[k] = gain
        first_order = (r + bd[k].T @ p[k+1] @ bd[k]) @ gain - bd[k].T @ p[k+1] @ ad[k]
        residuals.append(float(np.max(np.abs(first_order))) / max(1., np.max(np.abs(p[k]))))
    relative_terminal = float(np.max(np.abs(terminal_residual))) / max(1., np.max(np.abs(terminal)))
    if (not np.isfinite(p).all() or not np.isfinite(gains).all()
            or np.min(np.linalg.eigvalsh(p)) <= 0
            or max(residuals + [relative_terminal]) > 1e-9
            or np.max(np.abs(terminal_poles)) >= 1):
        raise RuntimeError("TVLQR failed positivity, optimality, or terminal stability checks")
    return TVLQRDesign(ad, bd, q, r, p, gains, terminal_gain, {
        "synthesis_status": "validated", "horizon_steps": len(ad),
        "terminal_dare_relative_residual": relative_terminal,
        "max_control_stationarity_relative_residual": max(residuals),
        "terminal_spectral_radius": float(np.max(np.abs(terminal_poles))),
        "minimum_value_matrix_eigenvalue": float(np.min(np.linalg.eigvalsh(p))),
    })


class TrajectoryController:
    """All three controllers output TOTAL Nm. No runner adds gravity afterwards."""
    names = ("joint_pd", "computed_torque_pd", "tvlqr")

    def __init__(self, name, dynamics, reference, design, settings):
        if name not in self.names:
            raise ValueError(f"Unknown controller {name}")
        self.name, self.dynamics, self.reference = name, dynamics, reference
        self.design, self.settings = design, settings
        omega = settings["pd"]["natural_frequency_rad_s"]
        zeta = settings["pd"]["damping_ratio"]
        self.acceleration_kp, self.acceleration_kd = omega**2, 2*zeta*omega
        # Fixed initial diagonal inertia gives independent, torque-domain PD gains.
        diag = np.diag(dynamics.mass(reference.q[0]))
        self.kp = diag * self.acceleration_kp
        self.kd = diag * self.acceleration_kd

    def compute(self, q, dq, index):
        ref = self.reference
        e = np.r_[q-ref.q[index], dq-ref.dq[index]]
        n = len(q)
        if self.name == "tvlqr":
            torque = ref.tau[index] - self.design.gains[index] @ e
        elif self.name == "joint_pd":
            torque = ref.tau[index] - self.kp*e[:n] - self.kd*e[n:]
        else:
            acceleration = ref.ddq[index] - self.acceleration_kp*e[:n] - self.acceleration_kd*e[n:]
            torque = self.dynamics.mass(q) @ acceleration + self.dynamics.bias(q, dq)
        if not np.isfinite(torque).all():
            raise RuntimeError("Nonfinite control output; stop this experiment")
        return torque
