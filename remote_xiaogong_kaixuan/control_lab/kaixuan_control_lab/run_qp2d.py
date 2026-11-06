"""Reproduce stage 01: reference checks, failure cases, CSVs, and figures."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import osqp
import scipy

from .config import ROOT
from .qp2d import (CONSTRAINT_NAMES, KKT_TOLERANCE, SOLVER_SETTINGS,
                   active_constraints, admm_trace, enumerate_reference, make_problem, solve_osqp)


SEED = 20261106
REFERENCE_TOLERANCE = 1e-6


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sample_row(scenario, index, reference):
    problem = make_problem(reference)
    independent = enumerate_reference(problem)
    result = solve_osqp(problem)
    if independent is None:
        raise AssertionError("The baseline feasible polygon has no reference optimum")
    valid = result.accepted
    return {
        "scenario": scenario, "index": index,
        "r1": float(reference[0]), "r2": float(reference[1]),
        "status": result.status, "accepted": valid, "rejection": result.rejection,
        "z1": float(result.z[0]) if valid else None,
        "z2": float(result.z[1]) if valid else None,
        "reference_z1": float(independent.z[0]), "reference_z2": float(independent.z[1]),
        "reference_error": float(np.max(np.abs(result.z - independent.z))) if valid else None,
        "objective": problem.objective(result.z) if valid else None,
        "active_constraints": "|".join(active_constraints(problem, result.z)) if valid else "",
        **{f"lambda_{name}": float(result.multipliers[i]) if valid else None
           for i, name in enumerate(CONSTRAINT_NAMES)},
        **{key: result.residuals[key] if result.residuals else None
           for key in ("primal_violation", "stationarity", "dual_violation", "complementarity")},
        **result.statistics,
    }


def draw_polygon(axis):
    polygon = np.array([[-1., -1.], [1., -1.], [1., -0.2], [-0.2, 1.], [-1., 1.]])
    axis.fill(*polygon.T, color="#e5edf4", edgecolor="#64748b", linewidth=1.4, label="Feasible set")
    axis.set(xlabel="$z_1$", ylabel="$z_2$", aspect="equal")
    axis.grid(alpha=0.2)


def plot_geometry(output):
    fig, axes = plt.subplots(2, 2, figsize=(10, 9), constrained_layout=True)
    mesh = np.linspace(-2.3, 2.5, 250)
    x, y = np.meshgrid(mesh, mesh)
    for axis, reference in zip(axes.flat, ([0., 0.], [1., 1.], [2., 1.], [-2., -2.])):
        problem = make_problem(reference)
        solution = enumerate_reference(problem)
        draw_polygon(axis)
        cost = (x - reference[0])**2 + 0.5 * (y - reference[1])**2
        axis.contour(x, y, cost, levels=[0.05, 0.2, 0.5, 1, 2, 3, 5, 8, 12],
                     colors="#c9b98a", linewidths=0.75)
        axis.plot(*reference, "x", color="#d97706", ms=10, mew=2, label="Target / unconstrained optimum")
        axis.plot(*solution.z, "o", color="#2563eb", ms=7, label="Constrained optimum")
        axis.annotate("", xy=solution.z, xytext=reference,
                      arrowprops={"arrowstyle": "->", "color": "#475569"})
        axis.set(xlim=(-2.3, 2.5), ylim=(-2.3, 2.5),
                 title=f"r={tuple(reference)}, z*=({solution.z[0]:.2f}, {solution.z[1]:.2f})")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncol=3, fontsize=9)
    fig.suptitle("Weighted distance: H=diag(2,1), box bounds and z1+z2 <= 0.8")
    fig.savefig(output / "geometry.png", dpi=160)
    plt.close(fig)


def plot_sweep(output, rows):
    rows = [row for row in rows if row["scenario"] == "sweep"]
    accepted = [row for row in rows if row["accepted"]]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.7), constrained_layout=True)
    draw_polygon(axes[0])
    axes[0].plot([r["r1"] for r in rows], [r["r2"] for r in rows], "--", color="#d97706", label="Target path")
    axes[0].plot([r["z1"] for r in accepted], [r["z2"] for r in accepted], color="#2563eb", label="OSQP path")
    axes[0].set_title("Target leaves the feasible set")
    axes[0].legend(fontsize=8)
    t = np.linspace(0, 1.6, len(rows))
    for coordinate, color in ((1, "#2563eb"), (2, "#d97706")):
        axes[1].plot(t, [r[f"r{coordinate}"] for r in rows], "--", color=color, alpha=0.5,
                     label=f"r{coordinate}")
        axes[1].plot([t[r["index"]] for r in accepted], [r[f"z{coordinate}"] for r in accepted],
                     color=color, label=f"z{coordinate} (OSQP)")
        axes[1].plot(t[::10], [r[f"reference_z{coordinate}"] for r in rows][::10],
                     "o", color=color, ms=3, fillstyle="none", label=f"z{coordinate} (reference)")
    axes[1].set(xlabel="t, with r=t*(1.5,1.2)", ylabel="Coordinate", title="Independent reference agrees")
    axes[1].legend(fontsize=8)
    for name in CONSTRAINT_NAMES:
        values = [r[f"lambda_{name}"] for r in accepted]
        if values and max(values) > 1e-7:
            axes[2].plot([t[r["index"]] for r in accepted], values, label=name)
    axes[2].set(xlabel="t", ylabel="Lagrange multiplier", title="Constraints become active")
    axes[2].legend(fontsize=8)
    for axis in axes[1:]:
        axis.grid(alpha=0.2)
    fig.savefig(output / "sweep.png", dpi=160)
    plt.close(fig)


def plot_admm(output, history):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    for key in ("primal_residual", "dual_residual"):
        axes[0].semilogy([row["iteration"] for row in history],
                         [max(row[key], 1e-16) for row in history], label=key)
    axes[0].set(xlabel="Iteration", ylabel="Infinity norm", title="Teaching ADMM, fixed rho=1")
    axes[0].legend()
    for coordinate, optimum in ((1, 0.6), (2, 0.2)):
        axes[1].plot([r["iteration"] for r in history], [r[f"z{coordinate}"] for r in history],
                     label=f"z{coordinate}")
        axes[1].axhline(optimum, linestyle="--", linewidth=0.8, color="#94a3b8")
    axes[1].set(xlabel="Iteration", ylabel="Coordinate", title="r=(1,1), optimum=(0.6,0.2)")
    axes[1].legend()
    for axis in axes:
        axis.grid(alpha=0.2)
    fig.savefig(output / "admm.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = args.output or ROOT / "artifacts" / f"qp2d_{stamp}"
    output.mkdir(parents=True, exist_ok=False)
    references = [("sweep", i, t * np.array([1.5, 1.2]))
                  for i, t in enumerate(np.linspace(0., 1.6, 101))]
    references += [("landmark", i, r) for i, r in enumerate(
        ([0., 0.], [1., 1.], [2., 1.], [-2., -2.], [2., -2.], [-2., 2.]))]
    rng = np.random.default_rng(SEED)
    references += [("random", i, r) for i, r in enumerate(rng.uniform(-3., 3., (128, 2)))]
    rows = [sample_row(*sample) for sample in references]
    write_csv(output / "solutions.csv", rows)
    failures = []
    for name, problem, options in (
        ("infeasible", make_problem([1., 1.], sum_upper=-2.5), {}),
        ("iteration_limit", make_problem([2., 2.]), {"max_iter": 1}),
    ):
        result = solve_osqp(problem, **options)
        failures.append({"case": name, "status": result.status, "status_value": result.status_value,
                         "accepted": result.accepted, "solution": None if result.z is None else result.z.tolist(),
                         "rejection": result.rejection, "sum_upper": float(problem.upper[-1]),
                         "reference": problem.reference.tolist(),
                         "reference_feasible": enumerate_reference(problem) is not None,
                         "max_iter": options.get("max_iter", SOLVER_SETTINGS["max_iter"]),
                         **result.statistics})
    (output / "failure_cases.json").write_text(json.dumps(failures, indent=2) + "\n", encoding="utf-8")
    history = admm_trace(make_problem([1., 1.]))
    write_csv(output / "admm_trace.csv", history)
    accepted = [row for row in rows if row["accepted"]]
    maximum_error = max((r["reference_error"] for r in accepted), default=None)
    maxima = {key: max((r[key] for r in accepted), default=None)
              for key in ("primal_violation", "stationarity", "dual_violation", "complementarity")}
    checks = {
        "all_samples_accepted": len(accepted) == len(rows),
        "reference_error_below_1e_6": maximum_error is not None and maximum_error < REFERENCE_TOLERANCE,
        "kkt_below_1e_7": all(v is not None and v <= KKT_TOLERANCE for v in maxima.values()),
        "infeasible_rejected": failures[0]["status_value"] == int(osqp.SolverStatus.OSQP_PRIMAL_INFEASIBLE)
                               and not failures[0]["accepted"] and failures[0]["solution"] is None
                               and not failures[0]["reference_feasible"],
        "iteration_limit_rejected": failures[1]["status_value"] == int(osqp.SolverStatus.OSQP_MAX_ITER_REACHED)
                                    and not failures[1]["accepted"] and failures[1]["solution"] is None,
        "teaching_admm_converged": max(abs(history[-1]["z1"] - .6), abs(history[-1]["z2"] - .2),
                                       history[-1]["primal_residual"], history[-1]["dual_residual"]) < 1e-7,
    }
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=False)
    problem = make_problem([0., 0.])
    summary = {
        "status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
        "seed": SEED, "samples": len(rows), "accepted_samples": len(accepted),
        "maximum_reference_error": maximum_error, "maximum_kkt_residuals": maxima,
        "versions": {"python": platform.python_version(), "platform": platform.platform(),
                     "numpy": np.__version__, "scipy": scipy.__version__, "osqp": osqp.__version__,
                     "matplotlib": matplotlib.__version__},
        "git_revision": revision.stdout.strip() if revision.returncode == 0 else None,
        "source_sha256": {name: hashlib.sha256((ROOT / "kaixuan_control_lab" / name).read_bytes()).hexdigest()
                          for name in ("qp2d.py", "run_qp2d.py", "config.py")},
        "problem": {"H": problem.hessian.tolist(), "C": problem.constraints.tolist(),
                    "d": problem.upper.tolist(), "constraint_names": CONSTRAINT_NAMES,
                    "sweep": {"r_direction": [1.5, 1.2], "t_range": [0., 1.6], "samples": 101},
                    "random": {"range": [-3., 3.], "samples": 128}},
        "solver_settings": SOLVER_SETTINGS,
        "acceptance": {"kkt_tolerance": KKT_TOLERANCE, "reference_tolerance": REFERENCE_TOLERANCE},
        "teaching_admm": {"rho": 1., "sigma": 1e-6, "iterations": len(history), "reference": [1., 1.]},
        "timing_scope": "Fresh OSQP workspace per sample; not a robot real-time guarantee",
        "wall_us_p50_p99_max": np.percentile([r["wall_us"] for r in rows], [50, 99, 100]).tolist(),
        "robot_model": "not used; dimensionless offline QP",
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    plot_geometry(output)
    plot_sweep(output, rows)
    plot_admm(output, history)
    print(json.dumps({k: summary[k] for k in ("status", "samples", "maximum_reference_error", "maximum_kkt_residuals")}, indent=2))
    print(f"Logs and figures: {output}")
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
