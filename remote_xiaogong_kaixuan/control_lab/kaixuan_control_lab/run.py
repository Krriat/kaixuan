"""Run an offline baseline, optionally with a local desktop viewer."""
import argparse
from contextlib import nullcontext
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from .backend import MujocoPlant
from .config import DEFAULT_CONFIG, ROOT, load_config, urdf_path
from .controllers import GravityController, ZeroTorqueController
from .model import build_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--controller", choices=("gravity", "zero"), default="gravity")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    config = load_config(args.config)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = args.output or ROOT / "artifacts" / stamp
    output.mkdir(parents=True, exist_ok=False)
    path = build_model(config, output / "model.xml", visuals=args.viewer)
    plant = MujocoPlant(path, config)
    controller = GravityController() if args.controller == "gravity" else ZeroTorqueController()
    controller.reset(plant.state())
    dt = config["physics_step_s"]
    viewer_context = nullcontext(None)
    if args.viewer:
        from mujoco import viewer as mj_viewer
        viewer_context = mj_viewer.launch_passive(plant.model, plant.data)
    labels = [f"{name}_{unit}" for unit in ("q_rad", "dq_rad_s", "requested_nm", "command_nm", "applied_nm")
              for name in config["joint_names"]]
    compute_us = []
    max_velocity = 0.0
    modified_commands = 0
    error = None
    stopped_early = False
    failure_state = None
    start = time.perf_counter()
    try:
        with (output / "trace.csv").open("w", newline="", encoding="utf-8") as stream, viewer_context as viewer:
            writer = csv.writer(stream)
            writer.writerow(["time_s", *labels, "compute_us"])
            for step in range(int(np.ceil(args.seconds / dt))):
                if viewer is not None and not viewer.is_running():
                    stopped_early = True
                    break
                state = plant.state()
                before = time.perf_counter_ns()
                tau = controller.compute(state, plant.dynamics)
                elapsed = (time.perf_counter_ns() - before) / 1000
                compute_us.append(elapsed)
                next_state = plant.step(tau)
                max_velocity = max(max_velocity, float(np.max(np.abs(next_state.dq))))
                modified_commands += int(not np.allclose(tau, plant.actuator.command, rtol=0, atol=1e-12))
                writer.writerow([state.time_s, *state.q, *state.dq, *tau,
                                 *plant.actuator.command, *plant.actuator.applied, elapsed])
                if viewer is not None:
                    if step % 16 == 0:
                        viewer.sync()
                    time.sleep(max(0, start + (step + 1) * dt - time.perf_counter()))
    except (RuntimeError, ValueError) as exc:
        error = str(exc)
        if np.isfinite(plant.data.qvel).all():
            max_velocity = max(max_velocity, float(np.max(np.abs(plant.data.qvel))))
        failure_state = {"q": plant.data.qpos.tolist(), "dq": plant.data.qvel.tolist(),
                         "requested_nm": np.asarray(tau).tolist() if "tau" in locals() else None}
    except KeyboardInterrupt:
        stopped_early = True
    finally:
        summary = {
            "status": "failed" if error else "stopped" if stopped_early else "completed",
            "error": error, "failure_state": failure_state,
            "backend": "mujoco_offline", "mujoco_version": mujoco.__version__,
            "controller": args.controller, "config": config,
            "urdf_sha256": hashlib.sha256(urdf_path(config).read_bytes()).hexdigest(),
            "simulated_seconds": float(plant.data.time), "wall_seconds": time.perf_counter() - start,
            "max_abs_velocity_rad_s": max_velocity, "modified_command_steps": modified_commands,
            "final_position_rad": plant.data.qpos.tolist(),
            "compute_us_p50_p99_max": np.percentile(compute_us, [50, 99, 100]).tolist() if compute_us else [],
        }
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({key: summary[key] for key in ("status", "error", "simulated_seconds", "max_abs_velocity_rad_s")}, indent=2))
        print(f"Logs: {output}")
    if error:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
