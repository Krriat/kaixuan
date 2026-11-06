"""Seven-axis Cartesian line tracking and hidden end-effector force disturbances."""
import argparse
import copy
import csv
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import time

import mujoco
import numpy as np
import scipy
import yaml

from .backend import MujocoPlant
from .config import ROOT, load_config, urdf_path, vector
from .lqr import TrajectoryController, design_tvlqr, linearize_dynamics, state_weights
from .lqr_experiment import run_case
from .model import build_model
from .trajectory import build_reference, integer_steps
from .validate import validate


def read_settings(path):
    settings = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if settings["schema_version"] != 2:
        raise ValueError("This experiment requires Cartesian LQR schema 2")
    config = load_config(ROOT / settings["base_config"])
    if config["joint_names"] != [f"left_joint{k}" for k in range(1, 8)]:
        raise ValueError("Seven active left-arm joints are required")
    config["initial_position_rad"] = np.deg2rad(vector(settings["initial_position_deg"], 7, "initial angles")).tolist()
    integer_steps(settings["control_period_s"], config["physics_step_s"], "control period")
    tr = settings["trajectory"]
    for key in ("start_hold_s", "move_s", "end_hold_s"):
        integer_steps(tr[key], settings["control_period_s"], key)
    if np.linalg.norm(vector(tr["displacement_world_m"], 3, "displacement")) <= 0:
        raise ValueError("The line must have nonzero length")
    for group in (settings["pd"], settings["acceptance"], settings["weights"]):
        for key, value in group.items():
            if not np.isfinite(value).all() or np.any(np.asarray(value) <= 0):
                raise ValueError(f"{key} must be finite and positive")
    vector(settings["weights"]["torque_scale_nm"], 7, "torque scales")
    if np.any(vector(settings["pd_frequency_sweep_rad_s"], len(settings["pd_frequency_sweep_rad_s"]), "PD frequencies") <= 0):
        raise ValueError("PD frequency sweep must be positive")
    if settings["strong_pd_frequency_rad_s"] not in settings["pd_frequency_sweep_rad_s"]:
        raise ValueError("The strong PD setting must also appear in the published sweep")
    required = settings.get("required_completion_controllers", list(TrajectoryController.names))
    if not required or len(set(required)) != len(required) or set(required)-set(TrajectoryController.names):
        raise ValueError("Required completion controllers must be a nonempty subset of available controllers")
    if "presentation" in settings:
        presentation = settings["presentation"]
        vector(presentation["camera_lookat_world_m"], 3, "replay camera target")
        for key in ("playback_speed", "camera_distance_m", "force_arrow_m_per_n"):
            if not np.isfinite(presentation[key]) or presentation[key] <= 0:
                raise ValueError(f"{key} must be positive")
    if (not np.isfinite(tr["ik_tolerance"]) or tr["ik_tolerance"] <= 0
            or type(tr["max_ik_iterations"]) is not int or tr["max_ik_iterations"] <= 0):
        raise ValueError("Invalid IK tolerances")
    pulse = settings["disturbance"]
    vector(pulse["force_world_n"], 3, "push force")
    if (not np.isfinite([pulse["start_s"], pulse["duration_s"]]).all()
            or pulse["duration_s"] <= 0 or pulse["start_s"] <= tr["start_hold_s"]
            or pulse["start_s"]+pulse["duration_s"] >= tr["start_hold_s"]+tr["move_s"]):
        raise ValueError("The finite pulse must occur during the motion")
    for case in settings["scenarios"].values():
        if (not np.isfinite([case["force_multiplier"], case["time_constant_s"]]).all()
                or type(case["delay_steps"]) is not int or case["delay_steps"] < 0 or case["time_constant_s"] < 0):
            raise ValueError("Invalid scenario")
    actuator = config["actuator"]
    if (actuator["delay_steps"] or actuator["time_constant_s"] or actuator["torque_slew_nm_s"] is not None
            or any(config["physical_friction"]["coulomb_nm"])
            or any(config["physical_friction"]["viscous_nm_s_rad"])):
        raise ValueError("The controller design model must be ideal and frictionless")
    return settings, config


def synthesize(dynamics, reference, settings):
    before = time.perf_counter()
    ads, bds, qs = [], [], []
    differential_checks = []
    for k in range(len(reference.times)-1):
        q, dq, tau = reference.q[k], reference.dq[k], reference.tau[k]
        a, b, ad, bd = linearize_dynamics(dynamics, q, dq, tau, settings["control_period_s"])
        ads.append(ad)
        bds.append(bd)
        qs.append(state_weights(dynamics, q, settings))
        if k % 100 == 0:
            from .lqr import forward_dynamics
            rng = np.random.default_rng(20261106+k)
            delta_x, delta_u = rng.normal(size=14), rng.normal(size=7)
            scale = 1e-6
            x = np.r_[q, dq]
            # Separate direction, scale, and central difference from column-wise synthesis.
            measured = (forward_dynamics(dynamics, x+scale*delta_x, tau+scale*delta_u)
                      - forward_dynamics(dynamics, x-scale*delta_x, tau-scale*delta_u))/(2*scale)
            predicted = a @ delta_x + b @ delta_u
            differential_checks.append(float(np.linalg.norm(measured-predicted)/max(1., np.linalg.norm(measured))))
    # The final sample is a static hold, so the last local model also supplies
    # the infinite-horizon terminal cost. The finite horizon is NOT an MPC loop.
    r = settings["control_period_s"] * settings["weights"]["effort_weight"] * np.diag(
        1/np.asarray(settings["weights"]["torque_scale_nm"])**2)
    design = design_tvlqr(np.array(ads), np.array(bds), np.array(qs), r)
    design.diagnostics.update(offline_design_wall_s=time.perf_counter()-before,
        linearization_directional_relative_error=max(differential_checks),
        state_order="[q1..q7 error rad; dq1..dq7 error rad/s]", input_order="total output-side Nm",
        nominal_reference_discretization="continuous spline / frozen local ZOH; no affine defect compensation")
    return design


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/cartesian_lqr_demo.yaml")
    parser.add_argument("--pinocchio", action="store_true", help="Require independent model validation")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--viewer", action="store_true", help="Replay the completed TVLQR push run locally")
    parser.add_argument("--video", action="store_true", help="Save a MuJoCo replay using system ffmpeg")
    args = parser.parse_args()
    settings, config = read_settings(args.config)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = (args.output or ROOT / "artifacts" / f"cartesian_lqr_{stamp}").resolve()
    output.mkdir(parents=True, exist_ok=False)
    pin_available = importlib.util.find_spec("pinocchio") is not None
    report = validate(config, compare_pinocchio=args.pinocchio or pin_available)
    (output / "model_validation.json").write_text(json.dumps(report, indent=2)+"\n")
    model_path = build_model(config, output / "model.xml", visuals=False)
    model_plant = MujocoPlant(model_path, config)
    dynamics = model_plant.dynamics
    reference = build_reference(dynamics, np.asarray(config["initial_position_rad"]),
        *model_plant.model.jnt_range.T, settings, config["physics_step_s"])
    if (np.any(np.max(np.abs(reference.tau), axis=0) >= config["limits"]["torque_nm"])
            or np.any(np.max(np.abs(reference.dq), axis=0) >= config["limits"]["velocity_rad_s"])):
        raise RuntimeError("Nominal reference exceeds configured actuator/state limits")
    print("Reference validated: seven joints, fixed orientation, Cartesian line", flush=True)
    design = synthesize(dynamics, reference, settings)
    np.savez_compressed(output / "design.npz", Ad=design.ad, Bd=design.bd, Q=design.q, R=design.r,
                        P=design.p, K=design.gains, K_terminal=design.terminal_gain)
    np.savez_compressed(output / "reference.npz", time_s=reference.times, q_rad=reference.q,
        dq_rad_s=reference.dq, ddq_rad_s2=reference.ddq, tau_nm=reference.tau,
        ee_position_m=reference.positions, ee_rotation=reference.orientation)
    (output / "design.json").write_text(json.dumps(design.diagnostics, indent=2)+"\n")
    runs = {}
    for scenario_name, scenario in settings["scenarios"].items():
        physical = copy.deepcopy(config)
        physical["actuator"].update(delay_steps=scenario["delay_steps"], time_constant_s=scenario["time_constant_s"])
        for name in TrajectoryController.names:
            key = f"{scenario_name}_{name}"
            controller = TrajectoryController(name, dynamics, reference, design, settings)
            summary, arrays = run_case(output / key, model_path, physical, controller, settings, scenario)
            summary["scenario_name"] = scenario_name
            (output / key / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
            runs[key] = summary, arrays
            print(f"{key}: {summary['status']}, peak={summary['peak_position_error_mm']:.3f} mm, "
                  f"final={summary['final_position_error_mm']:.4f} mm", flush=True)
    # Publish all sweep results instead of choosing weak PD gains after observing
    # the main disturbance. The common cost and motion/torque limits stay fixed.
    sweep = {}
    for frequency in settings["pd_frequency_sweep_rad_s"]:
        for scenario_name in ("nominal", "push"):
            for name in ("joint_pd", "computed_torque_pd"):
                if frequency == settings["pd"]["natural_frequency_rad_s"]:
                    result = runs[f"{scenario_name}_{name}"][0].copy()
                else:
                    variant = copy.deepcopy(settings)
                    variant["pd"]["natural_frequency_rad_s"] = frequency
                    controller = TrajectoryController(name, dynamics, reference, design, variant)
                    key = f"{scenario_name}_{name}_w{frequency:g}"
                    result, _ = run_case(output / "pd_sweep" / key, model_path, config, controller,
                                         settings, settings["scenarios"][scenario_name])
                result.update(pd_frequency_rad_s=frequency, scenario_name=scenario_name)
                sweep[f"{scenario_name}_{name}_w{frequency:g}"] = result
    (output / "pd_sweep.json").write_text(json.dumps(sweep, indent=2, allow_nan=False)+"\n")
    strong_runs = {}
    frequency = settings["strong_pd_frequency_rad_s"]
    for scenario_name, scenario in settings["scenarios"].items():
        if scenario_name in ("nominal", "push"):
            key = f"{scenario_name}_computed_torque_pd_w{frequency:g}"
            result = sweep[key].copy()
            folder = output / "pd_sweep" / key
            with np.load(folder / "trace.npz") as saved:
                arrays = {key: saved[key] for key in saved.files}
        else:
            variant = copy.deepcopy(settings)
            variant["pd"]["natural_frequency_rad_s"] = frequency
            physical = copy.deepcopy(config)
            physical["actuator"].update(delay_steps=scenario["delay_steps"], time_constant_s=scenario["time_constant_s"])
            controller = TrajectoryController("computed_torque_pd", dynamics, reference, design, variant)
            folder = output / "strong_pd" / scenario_name
            result, arrays = run_case(folder, model_path, physical, controller, settings, scenario)
            result.update(pd_frequency_rad_s=frequency, scenario_name=scenario_name)
            (folder / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
        result["trace_folder"] = str(folder.relative_to(output))
        strong_runs[scenario_name] = result, arrays
        print(f"Strong computed-torque PD w={frequency:g}, {scenario_name}: {result['status']}, "
              f"peak={result['peak_position_error_mm']:.3f} mm", flush=True)
    from .lqr_plots import plot_results
    plot_results(output, runs, reference, design, settings, sweep, strong_runs)
    regular = {key: run for key, run in runs.items() if not key.startswith("stress_")}
    required_controllers = settings.get("required_completion_controllers", list(TrajectoryController.names))
    required_runs = {key: run for key, run in regular.items() if run[0]["controller"] in required_controllers}
    checks = {
        "model_validation": report["status"] == "PASS",
        "reference_geometry": reference.diagnostics["max_position_error_m"] < settings["acceptance"]["reference_position_m"],
        "multiple_joints_move": sum(np.array(reference.diagnostics["joint_excursion_deg"]) > .1) >= 3,
        "required_regular_cases_complete": all(s["status"] == "completed" for s, _ in required_runs.values()),
        "required_regular_final_positions": all(s["final_position_error_mm"] < 1000*settings["acceptance"]["final_position_m"] for s, _ in required_runs.values()),
        "required_nominal_tracking": all(s["peak_position_error_mm"] < 1000*settings["acceptance"]["nominal_peak_position_m"]
            for key, (s, _) in required_runs.items() if key.startswith("nominal_")),
        "required_push_tracking": all(s["peak_position_error_mm"] < 1000*settings["acceptance"]["pushed_peak_position_m"] for s, _ in required_runs.values()),
        "motor_torque_limits": all(s["max_applied_torque_violation_nm"] <= 1e-12 for s, _ in runs.values()),
        "required_regular_state_limits": all(s["max_position_violation_rad"] == 0 and s["max_velocity_violation_rad_s"] == 0 for s, _ in required_runs.values()),
        "stress_outcomes_recorded": all(s["status"] in ("completed", "guard_stopped") for s, _ in runs.values()),
        "pd_sweep_outcomes_recorded": all(s["status"] in ("completed", "guard_stopped") for s in sweep.values()),
        "strong_pd_outcomes_recorded": all(s["status"] in ("completed", "guard_stopped") for s, _ in strong_runs.values()),
        "linearization": design.diagnostics["linearization_directional_relative_error"] < settings["acceptance"]["linearization_relative_error"],
        "riccati": max(design.diagnostics["terminal_dare_relative_residual"],
            design.diagnostics["max_control_stationarity_relative_residual"]) < settings["acceptance"]["riccati_relative_residual"],
    }
    source_files = [Path(__file__), Path(args.config), ROOT / "kaixuan_control_lab/lqr.py",
        ROOT / "kaixuan_control_lab/lqr_experiment.py", ROOT / "kaixuan_control_lab/trajectory.py",
        ROOT / "kaixuan_control_lab/lqr_plots.py", ROOT / "kaixuan_control_lab/lqr_replay.py",
        ROOT / "kaixuan_control_lab/lqr_video.py", ROOT / "kaixuan_control_lab/backend.py",
        ROOT / "kaixuan_control_lab/model.py", ROOT / "kaixuan_control_lab/config.py"]
    provenance = {str(p.resolve().relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}
    checks = {name: bool(passed) for name, passed in checks.items()}
    summary = {"status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
        "cross_engine_status": "PASS" if args.pinocchio or pin_available else "UNTESTED",
        "required_completion_controllers": required_controllers,
        "thread_environment": {key: os.environ.get(key) for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS")},
        "versions": {"python": platform.python_version(), "mujoco": mujoco.__version__, "numpy": np.__version__,
                     "scipy": scipy.__version__, "pinocchio": report["pinocchio_version"]},
        "source_sha256": provenance, "urdf_sha256": hashlib.sha256(urdf_path(config).read_bytes()).hexdigest(),
        "settings": settings, "controller_model_config": config, "reference": reference.diagnostics,
        "design": design.diagnostics, "cases": {key: value[0] for key, value in runs.items()}, "pd_frequency_sweep": sweep,
        "strong_computed_torque_pd": {key: value[0] for key, value in strong_runs.items()},
        "limitations": ["Offline rigid-body free-space simulation, no collision/contact or measured motor model",
            "J7 armature is estimated; J1-J6 rotor inertias are not identified",
            "PD baselines are simulated controllers, not vendor CSP or FOC",
            "LQR is locally optimal for the linearized unconstrained model; clipping/delay removes that guarantee",
            "The push is a world-frame prescribed force at the actual tool origin, not a modeled contact"]}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    flat_keys = ["controller", "scenario_name", "status", "peak_position_error_mm", "rms_position_error_mm",
        "final_position_error_mm", "recovery_after_pulse_s", "common_sampled_cost", "peak_velocity_rad_s", "torque_saturated_steps"]
    with (output / "metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=flat_keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(value[0] for value in runs.values())
    for filename, records in (("pd_sweep.csv", sweep.values()),
                              ("strong_pd_metrics.csv", [value[0] for value in strong_runs.values()])):
        with (output / filename).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=flat_keys+["pd_frequency_rad_s"], extrasaction="ignore")
            writer.writeheader()
            writer.writerows(records)
    print(f"{summary['status']}: {output}", flush=True)
    if args.video or args.viewer:
        from .lqr_replay import replay
        replay(output, config, settings, video=args.video, viewer=args.viewer)
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
