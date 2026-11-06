"""Repeatable seven-axis trajectory runs; the external push is hidden from control."""
from collections import deque
import csv
import json
from pathlib import Path
import time

import numpy as np

from .backend import MujocoPlant
from .trajectory import integer_steps, rotation_error


def disturbance_wrench(time_s, settings, multiplier):
    pulse = settings["disturbance"]
    phase = (time_s-pulse["start_s"]) / pulse["duration_s"]
    force = np.zeros(3)
    if pulse["start_s"] <= time_s < pulse["start_s"]+pulse["duration_s"]:
        force = multiplier * np.asarray(pulse["force_world_n"]) * np.sin(np.pi*phase)**2
    return np.r_[force, np.zeros(3)]


def recovery_time(times, error_norm, pulse_end, band):
    eligible = np.flatnonzero(times >= pulse_end - 1e-10)
    if not len(eligible):
        return None
    outside = eligible[error_norm[eligible] > band]
    first = eligible[0] if not len(outside) else outside[-1]+1
    return None if first >= len(times) else float(max(0., times[first]-pulse_end))


def run_case(output, model_path, config, controller, settings, scenario):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    plant = MujocoPlant(model_path, config)
    ref = controller.reference
    plant.reset(q=ref.q[0], dq=ref.dq[0])
    # Each run starts at the same prepared static equilibrium, including actuator
    # state and delay queue. This is NOT an implementation of hardware startup.
    initial_torque = plant.dynamics.gravity(ref.q[0])
    plant.actuator.command = initial_torque.copy()
    plant.actuator.applied = initial_torque.copy()
    plant.actuator.queue = deque(initial_torque.copy() for _ in range(plant.actuator.delay_steps))
    dt = config["physics_step_s"]
    stride = integer_steps(settings["control_period_s"], dt, "control period")
    steps = integer_steps(ref.times[-1], dt, "duration")
    q, dq, times = [plant.state().q], [plant.state().dq], [0.]
    positions, orientations = [], []
    p, rot, _ = plant.dynamics.kinematics(q[0])
    positions.append(p)
    orientations.append(float(np.linalg.norm(rotation_error(ref.orientation, rot))))
    requested, commands, applied, forces, external_torques = [], [], [], [], []
    flags, updates, timings = [], [], []
    error, error_kind = None, None
    torque = initial_torque.copy()
    sampled_cost = 0.
    for step in range(steps):
        state = plant.state()
        updated = step % stride == 0
        index = step // stride
        wrench = disturbance_wrench(step*dt, settings, scenario["force_multiplier"])
        elapsed = float("nan")
        try:
            if updated:
                before = time.perf_counter_ns()
                torque = controller.compute(state.q, state.dq, index)
                elapsed = (time.perf_counter_ns()-before)/1000
            plant.step(torque, external_wrench_world=wrench)
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as exc:
            error, error_kind = str(exc), "guard_stopped" if "guard" in str(exc) else "failed"
        next_state = plant.state()
        q.append(next_state.q)
        dq.append(next_state.dq)
        times.append(next_state.time_s)
        p, rot, _ = plant.dynamics.kinematics(next_state.q)
        positions.append(p)
        orientations.append(float(np.linalg.norm(rotation_error(ref.orientation, rot))))
        requested.append(torque.copy())
        commands.append(plant.actuator.command.copy())
        applied.append(plant.actuator.applied.copy())
        forces.append(wrench[:3])
        external_torques.append(plant.data.qfrc_applied.copy())
        flags.append(bool(np.max(np.abs(torque)) > 0 and np.any(np.abs(torque) > plant.actuator.limit+1e-12)))
        updates.append(updated)
        timings.append(elapsed)
        if updated:
            e = np.r_[state.q-ref.q[index], state.dq-ref.dq[index]]
            du = plant.actuator.applied-ref.tau[index]
            sampled_cost += float(e @ controller.design.q[index] @ e + du @ controller.design.r @ du)
        if error is not None:
            break
    arrays = {"time_s": np.asarray(times), "q_rad": np.asarray(q), "dq_rad_s": np.asarray(dq),
        "ee_position_m": np.asarray(positions), "orientation_error_rad": np.asarray(orientations),
        "requested_nm": np.asarray(requested), "command_nm": np.asarray(commands),
        "applied_nm": np.asarray(applied), "force_world_n": np.asarray(forces),
        "external_generalized_nm": np.asarray(external_torques),
        "torque_saturated": np.asarray(flags), "control_updated": np.asarray(updates),
        "compute_us": np.asarray(timings)}
    arrays["desired_position_m"] = ref.desired_position(arrays["time_s"])
    arrays["reference_q_rad"], arrays["reference_dq_rad_s"], _ = ref.sample(arrays["time_s"])
    norm = np.linalg.norm(arrays["ee_position_m"]-arrays["desired_position_m"], axis=1)
    q_error = arrays["q_rad"]-arrays["reference_q_rad"]
    pulse = settings["disturbance"]
    after_push = arrays["time_s"] >= pulse["start_s"]
    complete = error is None
    if complete:
        e_final = np.r_[q_error[-1], arrays["dq_rad_s"][-1]-ref.dq[-1]]
        sampled_cost += float(e_final @ controller.design.p[-1] @ e_final)
    compute = arrays["compute_us"][np.isfinite(arrays["compute_us"])]
    lower, upper = plant.model.jnt_range.T
    violations_q = np.maximum(np.maximum(lower-arrays["q_rad"], arrays["q_rad"]-upper), 0.)
    violations_v = np.maximum(np.abs(arrays["dq_rad_s"])-np.asarray(config["limits"]["velocity_rad_s"]), 0.)
    summary = {
        "status": "completed" if complete else error_kind, "error": error,
        "controller": controller.name, "scenario": scenario,
        "simulated_seconds": float(times[-1]), "physics_step_s": dt,
        "control_period_s": settings["control_period_s"],
        "initial_actuator_torque_nm": initial_torque.tolist(),
        "peak_position_error_mm": float(1000*max(norm)),
        "rms_position_error_mm": float(1000*np.sqrt(np.mean(norm**2))),
        "post_push_rms_position_error_mm": float(1000*np.sqrt(np.mean(norm[after_push]**2))) if np.any(after_push) else None,
        "final_position_error_mm": float(1000*norm[-1]),
        "peak_orientation_error_deg": float(np.rad2deg(max(orientations))),
        "peak_joint_error_deg": float(np.rad2deg(np.max(np.abs(q_error)))),
        "peak_velocity_rad_s": float(np.max(np.abs(arrays["dq_rad_s"]))),
        "peak_requested_nm_by_joint": np.max(np.abs(arrays["requested_nm"]), axis=0).tolist(),
        "peak_applied_nm_by_joint": np.max(np.abs(arrays["applied_nm"]), axis=0).tolist(),
        "torque_rms_nm_by_joint": np.sqrt(np.mean(arrays["applied_nm"]**2, axis=0)).tolist(),
        "torque_saturated_steps": int(sum(flags)),
        "control_updates": int(sum(updates)),
        "common_sampled_cost": sampled_cost, "cost_is_truncated": not complete,
        "compute_us_p50_p99_max": np.percentile(compute, [50, 99, 100]).tolist() if len(compute) else [],
        "recovery_after_pulse_s": recovery_time(arrays["time_s"], norm, pulse["start_s"]+pulse["duration_s"],
            settings["acceptance"]["recovery_band_m"]) if complete and scenario["force_multiplier"] else None,
        "max_position_violation_rad": float(np.max(violations_q)),
        "max_velocity_violation_rad_s": float(np.max(violations_v)),
        "max_applied_torque_violation_nm": float(np.max(np.maximum(np.abs(arrays["applied_nm"])-plant.actuator.limit, 0.))),
        "measured_force_impulse_ns": (np.sum(arrays["force_world_n"], axis=0)*dt).tolist(),
        "failure_state": None if complete else {"time_s": float(times[-1]), "q_rad": q[-1].tolist(),
            "dq_rad_s": dq[-1].tolist(), "requested_nm": torque.tolist()},
    }
    np.savez_compressed(output / "trace.npz", **arrays)
    labels = ["time_s", "next_time_s", "control_updated", "compute_us", "torque_saturated", "position_error_mm"]
    vector_fields = [("q_rad", 7), ("dq_rad_s", 7), ("reference_q_rad", 7), ("reference_dq_rad_s", 7),
        ("requested_nm", 7), ("command_nm", 7), ("applied_nm", 7), ("external_generalized_nm", 7),
        ("ee_position_m", 3), ("desired_position_m", 3), ("force_world_n", 3)]
    labels += [f"{name}_{j+1}" for name, size in vector_fields for j in range(size)]
    with (output / "trace.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(labels)
        for k in range(len(requested)):
            row = [times[k], times[k+1], updates[k], timings[k], flags[k], 1000*norm[k]]
            for name, _ in vector_fields:
                row.extend(arrays[name][k])
            writer.writerow(row)
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return summary, arrays
