"""Stage 01: a strictly convex 2-D QP, an independent reference, and ADMM.

These variables are dimensionless teaching examples, not robot commands.
Only a solved status AND independently checked KKT conditions expose a solution.
"""
from dataclasses import dataclass
from itertools import combinations
import time

import numpy as np
import osqp
from scipy import sparse

from .config import vector


CONSTRAINT_NAMES = ("z1_upper", "z1_lower", "z2_upper", "z2_lower", "sum_upper")
SOLVER_SETTINGS = {
    "eps_abs": 1e-9,
    "eps_rel": 1e-9,
    "max_iter": 10000,
    "check_termination": 1,
    "adaptive_rho_interval": 50,
    "polishing": False,
    "warm_starting": False,
    "verbose": False,
}
KKT_TOLERANCE = 1e-7


@dataclass(frozen=True)
class Problem:
    reference: np.ndarray
    hessian: np.ndarray
    constraints: np.ndarray
    upper: np.ndarray

    def objective(self, z):
        error = vector(z, 2, "z") - self.reference
        return float(0.5 * error @ self.hessian @ error)


def make_problem(reference, *, sum_upper=0.8):
    if not np.isfinite(sum_upper):
        raise ValueError("sum_upper must be finite")
    return Problem(
        reference=vector(reference, 2, "reference").copy(),
        hessian=np.diag([2.0, 1.0]),
        constraints=np.array([[1., 0.], [-1., 0.], [0., 1.], [0., -1.], [1., 1.]]),
        upper=np.array([1., 1., 1., 1., sum_upper]),
    )


def kkt_residuals(problem, z, multipliers):
    z = vector(z, 2, "z")
    multipliers = vector(multipliers, len(problem.upper), "multipliers")
    slack = problem.constraints @ z - problem.upper
    stationarity = problem.hessian @ (z - problem.reference) + problem.constraints.T @ multipliers
    return {
        "primal_violation": float(max(0., np.max(slack))),
        "stationarity": float(np.max(np.abs(stationarity))),
        "dual_violation": float(max(0., -np.min(multipliers))),
        "complementarity": float(np.max(np.abs(multipliers * slack))),
    }


def active_constraints(problem, z, *, tolerance=1e-7):
    return [name for name, slack in zip(CONSTRAINT_NAMES, problem.upper - problem.constraints @ z)
            if abs(slack) <= tolerance]


@dataclass(frozen=True)
class ReferenceSolution:
    z: np.ndarray
    multipliers: np.ndarray


def enumerate_reference(problem):
    """Enumerate zero, one, or two independent active normals; no QP solver.

    A 2-D convex polygon optimum has a KKT representation using at most two
    independent normals. Singular subsets are skipped; all inequalities and
    multiplier signs are checked before comparing original objective values.
    """
    best = None
    for count in range(3):
        for indices in combinations(range(len(problem.upper)), count):
            active = problem.constraints[list(indices)]
            matrix = np.block([[problem.hessian, active.T],
                               [active, np.zeros((count, count))]])
            rhs = np.r_[problem.hessian @ problem.reference, problem.upper[list(indices)]]
            try:
                candidate = np.linalg.solve(matrix, rhs)
            except np.linalg.LinAlgError:
                continue
            z = candidate[:2]
            multipliers = np.zeros(len(problem.upper))
            multipliers[list(indices)] = candidate[2:]
            if max(kkt_residuals(problem, z, multipliers).values()) > 1e-9:
                continue
            if best is None or problem.objective(z) < problem.objective(best.z):
                best = ReferenceSolution(z.copy(), multipliers.copy())
    return best  # None means no feasible optimum; never substitute a zero vector.


@dataclass(frozen=True)
class SolverResult:
    status: str
    status_value: int
    accepted: bool
    rejection: str | None
    z: np.ndarray | None
    multipliers: np.ndarray | None
    residuals: dict | None
    statistics: dict


def solve_osqp(problem, *, max_iter=SOLVER_SETTINGS["max_iter"]):
    """A fresh workspace per solve: timings are cold solves, not a control budget."""
    if type(max_iter) is not int or max_iter <= 0:
        raise ValueError("max_iter must be a positive integer")
    start = time.perf_counter_ns()
    solver = osqp.OSQP()
    solver.setup(P=sparse.csc_matrix(problem.hessian),
                 q=-problem.hessian @ problem.reference,
                 A=sparse.csc_matrix(problem.constraints),
                 l=np.full(len(problem.upper), -np.inf), u=problem.upper,
                 **{**SOLVER_SETTINGS, "max_iter": max_iter})
    raw = solver.solve(raise_error=False)
    info = raw.info
    statistics = {
        "iterations": int(info.iter),
        "osqp_primal_residual": float(info.prim_res),
        "osqp_dual_residual": float(info.dual_res),
        "setup_us": float(info.setup_time * 1e6),
        "solve_us": float(info.solve_time * 1e6),
        "run_us": float(info.run_time * 1e6),
        "wall_us": (time.perf_counter_ns() - start) / 1000,
    }
    rejection = None
    z = multipliers = residuals = None
    if info.status_val != int(osqp.SolverStatus.OSQP_SOLVED):
        rejection = "solver status is not solved"
    else:
        try:
            candidate = vector(raw.x, 2, "solver solution")
            dual = vector(raw.y, len(problem.upper), "solver multipliers")
            residuals = kkt_residuals(problem, candidate, dual)
            if not np.isfinite(list(residuals.values())).all() or max(residuals.values()) > KKT_TOLERANCE:
                rejection = "independent KKT check failed"
            else:
                z, multipliers = candidate.copy(), dual.copy()
        except (TypeError, ValueError):
            rejection = "solver returned nonfinite or malformed values"
    return SolverResult(info.status, int(info.status_val), rejection is None,
                        rejection, z, multipliers, residuals, statistics)


def admm_trace(problem, *, iterations=300, rho=1.0, sigma=1e-6):
    """Unrelaxed, fixed-rho teaching ADMM; NOT OSQP's internal iteration trace.

    Split v=Cz, v<=d. The z step is a proximal quadratic minimization, the v
    step projects onto (-inf,d], and y accumulates the consistency error.
    OSQP additionally uses scaling, relaxation, adaptive rho, and sparse solves.
    """
    if type(iterations) is not int or iterations <= 0:
        raise ValueError("iterations must be a positive integer")
    if not np.isfinite([rho, sigma]).all() or rho <= 0 or sigma <= 0:
        raise ValueError("rho and sigma must be finite and positive")
    h, c, d = problem.hessian, problem.constraints, problem.upper
    z = np.zeros(2)
    v = np.minimum(c @ z, d)
    y = np.zeros(len(d))
    matrix = h + sigma * np.eye(2) + rho * c.T @ c
    history = []
    for iteration in range(1, iterations + 1):
        z = np.linalg.solve(matrix, sigma * z + h @ problem.reference + c.T @ (rho * v - y))
        cz = c @ z
        v = np.minimum(cz + y / rho, d)
        y += rho * (cz - v)
        history.append({
            "iteration": iteration, "z1": float(z[0]), "z2": float(z[1]),
            "objective": problem.objective(z),
            "primal_residual": float(np.max(np.abs(cz - v))),
            "dual_residual": float(np.max(np.abs(h @ (z - problem.reference) + c.T @ y))),
            **kkt_residuals(problem, z, y),
        })
    return history
