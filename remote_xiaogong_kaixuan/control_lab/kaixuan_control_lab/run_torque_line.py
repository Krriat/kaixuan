"""Teaching demo: TOTAL joint torque drives a fixed-orientation WORLD +Z line.

Start at TARGET_JOINTS_DEG, hold 1 s, move up 0.10 m in 4 s, hold 1 s.
Only the initial condition sets qpos. During motion the plant receives Nm only.
No LQR synthesis, MPC, external push, ROS or robot transport is involved.
"""
import argparse
from contextlib import contextmanager, nullcontext
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

from .backend import MujocoPlant
from .config import ROOT, load_config, urdf_path
from .model import build_model
from .trajectory import build_reference, integer_steps, rotation_error


TARGET_JOINTS_DEG = [-20, 20, 30, 50, -10, -20, 0]
DISPLACEMENT_WORLD_M = [0.0, 0.0, 0.10]
NATURAL_FREQUENCY_RAD_S = 12.0
DAMPING_RATIO = 1.0


def computed_torque(dynamics, q, dq, q_ref, dq_ref, ddq_ref):
    """Feedback is in acceleration units; M converts it to joint torque.

    tau = M(q) [ddq_ref + Kp(q_ref-q) + Kd(dq_ref-dq)] + c(q,dq) + G(q)
    Kp=144/s^2 and Kd=24/s. These are control gains, not physical friction.
    """
    kp = NATURAL_FREQUENCY_RAD_S**2
    kd = 2 * DAMPING_RATIO * NATURAL_FREQUENCY_RAD_S
    acceleration = ddq_ref + kp * (q_ref - q) + kd * (dq_ref - dq)
    return dynamics.mass(q) @ acceleration + dynamics.bias(q, dq)


def prepare(output, *, move_seconds=4.0, visuals=False, dynamics_name="mujoco"):
    config = load_config()
    if config["joint_names"] != [f"left_joint{i}" for i in range(1, 8)]:
        raise ValueError("This demo requires all seven left-arm joints in order")
    # Keep the first lesson about motion control, with the ideal actuator baseline.
    motor = config["actuator"]
    if (motor["delay_steps"] or motor["time_constant_s"] or motor["torque_slew_nm_s"] is not None
            or any(config["physical_friction"]["viscous_nm_s_rad"])
            or any(config["physical_friction"]["coulomb_nm"])):
        raise ValueError("Use the ideal, frictionless left_arm.yaml baseline for this lesson")
    config["initial_position_rad"] = np.deg2rad(TARGET_JOINTS_DEG).tolist()
    settings = {
        # Reference knots every 10 ms; feedback and physics every 1 ms.
        "control_period_s": 0.01,
        "trajectory": {"start_hold_s": 1.0, "move_s": move_seconds, "end_hold_s": 1.0,
                       "displacement_world_m": DISPLACEMENT_WORLD_M,
                       "ik_tolerance": 1e-10, "max_ik_iterations": 40},
        "acceptance": {"reference_position_m": 1e-6, "reference_orientation_rad": 1e-6},
    }
    integer_steps(move_seconds, settings["control_period_s"], "move duration")
    model_path = build_model(config, Path(output) / "model.xml", visuals=visuals)
    plant = MujocoPlant(model_path, config)
    plant.check_state()
    if dynamics_name == "pinocchio":
        from .pinocchio_model import PinocchioDynamics
        dynamics = PinocchioDynamics(config)
    elif dynamics_name == "mujoco":
        dynamics = plant.dynamics
    else:
        raise ValueError("Unknown dynamics provider")
    reference = build_reference(dynamics, plant.state().q, *plant.model.jnt_range.T,
                                settings, config["physics_step_s"])
    diagnostics = reference.diagnostics
    if (np.any(np.asarray(diagnostics["peak_reference_velocity_rad_s"]) >= config["limits"]["velocity_rad_s"])
            or np.any(np.asarray(diagnostics["peak_reference_torque_nm"]) >= config["limits"]["torque_nm"])):
        raise RuntimeError("Reference exceeds torque/speed limits; increase move duration or revise path")
    return config, plant, dynamics, reference


def draw_path(viewer, desired, actual, start, end):
    """Draw the requested segment and current desired/actual tool positions."""
    scene = viewer.user_scn
    scene.ngeom = 0
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                       np.eye(3).ravel(), np.array([.1, .9, .3, 1.], dtype=np.float32))
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, .0015, start, end)
    scene.ngeom += 1
    for position, radius, color in ((end, .005, [.1, .9, .3, 1.]),
                                    (desired, .006, [1., .75, .1, 1.]),
                                    (actual, .004, [.1, .4, 1., 1.])):
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                           np.full(3, radius), position, np.eye(3).ravel(),
                           np.asarray(color, dtype=np.float32))
        scene.ngeom += 1


@contextmanager
def live_viewer(plant, reference, enabled):
    if not enabled:
        yield None
        return
    from mujoco import viewer as mj_viewer
    plant.model.vis.headlight.ambient[:] = [.45, .45, .45]
    for geom in range(plant.model.ngeom):
        if plant.model.geom_bodyid[geom] != 0:
            plant.model.geom_rgba[geom] = [.3, .6, .9, 1.]
        elif plant.model.geom(geom).name != "ground_visual":
            plant.model.geom_rgba[geom] = [.55, .60, .66, .18]
    existing_threads = set(threading.enumerate())
    window = mj_viewer.launch_passive(plant.model, plant.data, show_left_ui=False, show_right_ui=False)
    owned_threads = set(threading.enumerate()) - existing_threads
    try:
        with window:
            with window.lock():
                window.cam.lookat[:] = reference.start_position + .5 * reference.displacement
                window.cam.distance, window.cam.azimuth, window.cam.elevation = 1.3, 90., -15.
            yield window
    finally:
        window.close()
        for thread in owned_threads:
            thread.join(timeout=5.)
        if any(thread.is_alive() for thread in owned_threads):
            raise RuntimeError("Viewer did not shut down cleanly")


def simulate(plant, dynamics, reference, *, viewer=None):
    """Closed-loop integration: read q,dq -> compute Nm -> plant.step(tau)."""
    dt = plant.config["physics_step_s"]
    steps = integer_steps(reference.times[-1], dt, "experiment duration")
    initial = plant.state()
    times, q, dq = [initial.time_s], [initial.q], [initial.dq]
    position, rotation, _ = plant.dynamics.kinematics(initial.q)
    positions, orientation_errors = [position], [np.linalg.norm(rotation_error(reference.orientation, rotation))]
    requested, commands, applied, actual_motor_torques, compute_us = [], [], [], [], []
    completed, error = False, None
    start_wall = time.perf_counter()
    try:
        for step in range(steps):
            if viewer is not None and not viewer.is_running():
                error = "Viewer closed before the motion finished"
                break
            state = plant.state()
            q_ref, dq_ref, ddq_ref = reference.sample(state.time_s)
            before = time.perf_counter_ns()
            tau = computed_torque(dynamics, state.q, state.dq, q_ref, dq_ref, ddq_ref)
            if not np.isfinite(tau).all():
                raise RuntimeError("Nonfinite torque")
            compute_us.append((time.perf_counter_ns() - before) / 1000)
            # This is the only command during motion. No qpos/qvel assignment.
            try:
                with viewer.lock() if viewer is not None else nullcontext():
                    plant.step(tau)
            except (RuntimeError, ValueError) as exc:
                error = str(exc)
            current = plant.state()
            times.append(current.time_s)
            q.append(current.q)
            dq.append(current.dq)
            position, rotation, _ = plant.dynamics.kinematics(current.q)
            positions.append(position)
            orientation_errors.append(np.linalg.norm(rotation_error(reference.orientation, rotation)))
            requested.append(tau)
            commands.append(plant.actuator.command.copy())
            applied.append(plant.actuator.applied.copy())
            actual_motor_torques.append(plant.data.qfrc_actuator.copy())
            if error:
                break
            if viewer is not None:
                if step % 16 == 0 or step == steps - 1:
                    with viewer.lock():
                        draw_path(viewer, reference.desired_position(current.time_s), position,
                                  reference.start_position, reference.start_position + reference.displacement)
                    viewer.sync()
                time.sleep(max(0., start_wall + (step + 1)*dt - time.perf_counter()))
        else:
            completed = True
    except (RuntimeError, ValueError, np.linalg.LinAlgError, KeyboardInterrupt) as exc:
        error = str(exc) or "Interrupted"
    arrays = {"time_s": np.array(times), "q_rad": np.array(q), "dq_rad_s": np.array(dq),
              "ee_position_m": np.array(positions), "orientation_error_rad": np.array(orientation_errors),
              "requested_nm": np.array(requested).reshape(-1, 7),
              "command_nm": np.array(commands).reshape(-1, 7),
              "applied_nm": np.array(applied).reshape(-1, 7),
              "motor_generalized_nm": np.array(actual_motor_torques).reshape(-1, 7)}
    arrays["desired_position_m"] = reference.desired_position(arrays["time_s"])
    arrays["reference_q_rad"], arrays["reference_dq_rad_s"], _ = reference.sample(arrays["time_s"])
    errors = np.linalg.norm(arrays["ee_position_m"] - arrays["desired_position_m"], axis=1)
    lateral = np.linalg.norm(arrays["ee_position_m"][:, :2] - reference.start_position[:2], axis=1)
    target = reference.start_position + reference.displacement
    final_error = float(np.linalg.norm(arrays["ee_position_m"][-1] - target))
    saturated = int(np.sum(np.any(np.abs(arrays["requested_nm"]) > plant.actuator.limit + 1e-12, axis=1)))
    checks = {"completed_motion": completed,
              "max_tracking_error_below_1mm": bool(np.max(errors) < .001),
              "max_lateral_error_below_1mm": bool(np.max(lateral) < .001),
              "final_error_below_0p1mm": final_error < .0001,
              "orientation_error_below_0p1deg": bool(np.max(orientation_errors) < np.deg2rad(.1)),
              "no_torque_saturation": saturated == 0}
    summary = {"status": "PASS" if all(checks.values()) else "FAIL", "error": error,
               "checks": checks, "simulated_seconds": float(times[-1]),
               "initial_joint_deg": np.rad2deg(q[0]).tolist(),
               "final_joint_deg": np.rad2deg(q[-1]).tolist(),
               "start_position_world_m": reference.start_position.tolist(),
               "target_position_world_m": target.tolist(),
               "final_position_world_m": positions[-1].tolist(),
               "actual_displacement_world_m": (positions[-1] - positions[0]).tolist(),
               "max_tracking_error_mm": float(1000*np.max(errors)),
               "max_lateral_error_mm": float(1000*np.max(lateral)),
               "final_error_mm": 1000*final_error,
               "max_orientation_error_deg": float(np.rad2deg(np.max(orientation_errors))),
               "peak_speed_deg_s": float(np.rad2deg(np.max(np.abs(arrays["dq_rad_s"])))),
               "peak_torque_nm": np.max(np.abs(arrays["applied_nm"]), axis=0).tolist() if applied else [],
               "torque_saturated_steps": saturated,
               "compute_us_p50_p99_max": np.percentile(compute_us, [50, 99, 100]).tolist() if compute_us else []}
    return summary, arrays


def save_results(output, summary, arrays):
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    np.savez_compressed(output / "trace.npz", **arrays)
    labels = [f"{name}_{axis}" for name in ("desired_m", "actual_m") for axis in "xyz"]
    with (output / "trace.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", *labels, *[f"q{i}_rad" for i in range(1, 8)],
                         *[f"dq{i}_rad_s" for i in range(1, 8)],
                         *[f"requested{i}_nm" for i in range(1, 8)],
                         *[f"applied{i}_nm" for i in range(1, 8)]])
        # Row k is the state at t_k and the torque used in [t_k,t_{k+1}).
        for k in range(len(arrays["applied_nm"])):
            writer.writerow([arrays["time_s"][k], *arrays["desired_position_m"][k],
                             *arrays["ee_position_m"][k], *arrays["q_rad"][k], *arrays["dq_rad_s"][k],
                             *arrays["requested_nm"][k], *arrays["applied_nm"][k]])
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t, p, desired = (arrays[key] for key in ("time_s", "ee_position_m", "desired_position_m"))
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    axes[0, 0].plot(t, 100*(desired[:, 2]-p[0, 2]), "--", label="desired")
    axes[0, 0].plot(t, 100*(p[:, 2]-p[0, 2]), label="actual")
    axes[0, 0].set_ylabel("World Z displacement (cm)")
    axes[0, 1].plot(t, 1000*(p[:, 0]-p[0, 0]), label="X deviation")
    axes[0, 1].plot(t, 1000*(p[:, 1]-p[0, 1]), label="Y deviation")
    axes[0, 1].set_ylabel("Lateral deviation (mm)")
    for i in range(7):
        axes[1, 0].plot(t[:len(arrays["applied_nm"])], arrays["applied_nm"][:, i], label=f"J{i+1}")
    axes[1, 0].set_ylabel("Applied joint torque (Nm)")
    axes[1, 1].plot(t, 1000*np.linalg.norm(p-desired, axis=1), label="position error")
    axes[1, 1].set_ylabel("Tracking error (mm)")
    for ax in axes.flat:
        ax.set_xlabel("Time (s)")
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    fig.suptitle("Torque-only Cartesian +Z 10 cm | computed torque + PD")
    fig.savefig(output / "tracking.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="Show live torque-driven simulation")
    parser.add_argument("--move-seconds", type=float, default=4.0)
    parser.add_argument("--dynamics", choices=("mujoco", "pinocchio"), default="mujoco")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    integer_steps(args.move_seconds, .01, "move duration")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = (args.output or ROOT / "artifacts" / f"torque_line_{stamp}").resolve()
    output.mkdir(parents=True, exist_ok=False)
    config, plant, dynamics, reference = prepare(output, move_seconds=args.move_seconds,
                                                visuals=args.viewer, dynamics_name=args.dynamics)
    print(f"Start joints (deg): {TARGET_JOINTS_DEG}", flush=True)
    print(f"World start (m): {reference.start_position}", flush=True)
    print(f"World target (m): {reference.start_position + reference.displacement}", flush=True)
    print("Hold 1 s -> WORLD +Z 10 cm -> hold 1 s; total joint torque, fixed tool orientation", flush=True)
    with live_viewer(plant, reference, args.viewer) as window:
        summary, arrays = simulate(plant, dynamics, reference, viewer=window)
    summary.update(config=config, dynamics=args.dynamics, mujoco_version=mujoco.__version__,
                   urdf_sha256=hashlib.sha256(urdf_path(config).read_bytes()).hexdigest(),
                   reference_diagnostics=reference.diagnostics,
                   move_seconds=args.move_seconds, feedback_period_s=config["physics_step_s"],
                   kp_s_minus2=NATURAL_FREQUENCY_RAD_S**2,
                   kd_s_minus1=2*DAMPING_RATIO*NATURAL_FREQUENCY_RAD_S)
    save_results(output, summary, arrays)
    print(json.dumps({key: summary[key] for key in ("status", "actual_displacement_world_m",
          "max_tracking_error_mm", "max_lateral_error_mm", "final_error_mm", "peak_speed_deg_s")}, indent=2))
    print(f"Results: {output}")
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
