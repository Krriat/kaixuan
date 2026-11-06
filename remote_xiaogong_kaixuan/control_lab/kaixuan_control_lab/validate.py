"""Numerical model validation, optionally against independent Pinocchio dynamics."""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile

import mujoco
import numpy as np

from .backend import MujocoPlant
from .config import DEFAULT_CONFIG, load_config, urdf_path
from .model import build_model


def validate(config, *, compare_pinocchio=False, samples=32):
    with tempfile.TemporaryDirectory(prefix="kaixuan-model-") as directory:
        path = build_model(config, Path(directory) / "model.xml", visuals=False)
        plant = MujocoPlant(path, config)
        dynamics = plant.dynamics
        other = None
        if compare_pinocchio:
            from .pinocchio_model import PinocchioDynamics
            other = PinocchioDynamics(config)
        n = plant.model.nv
        errors = {key: 0.0 for key in ("mass", "gravity", "bias", "position", "rotation", "jacobian", "inverse_dynamics")}
        min_eigenvalue = float("inf")
        rng = np.random.default_rng(20261105)
        lower, upper = plant.model.jnt_range.T
        for _ in range(samples):
            q = rng.uniform(0.4 * lower, 0.4 * upper)
            dq = rng.uniform(-0.3, 0.3, n)
            ddq = rng.uniform(-0.5, 0.5, n)
            mass = dynamics.mass(q)
            min_eigenvalue = min(min_eigenvalue, float(np.linalg.eigvalsh(mass)[0]))
            if other is not None:
                pairs = {"mass": (mass, other.mass(q)),
                         "gravity": (dynamics.gravity(q), other.gravity(q)),
                         "bias": (dynamics.bias(q, dq), other.bias(q, dq)),
                         "inverse_dynamics": (mass @ ddq + dynamics.bias(q, dq),
                                               other.inverse_dynamics(q, dq, ddq))}
                for key, a, b in zip(("position", "rotation", "jacobian"),
                                     dynamics.kinematics(q), other.kinematics(q)):
                    pairs[key] = a, b
                for key, (a, b) in pairs.items():
                    if np.shape(a) != np.shape(b):
                        raise AssertionError(f"MuJoCo / Pinocchio {key} shape mismatch: {np.shape(a)} vs {np.shape(b)}")
                    errors[key] = max(errors[key], float(np.max(np.abs(a - b))))
        if min_eigenvalue <= 0:
            raise AssertionError("Non-positive mass matrix")
        if other is not None and any(value > 1e-8 for value in errors.values()):
            raise AssertionError(f"MuJoCo / Pinocchio disagreement: {errors}")
        # Gravity equilibrium from rest, not a physical model accuracy claim.
        q0 = plant.state().q
        for _ in range(round(1.0 / config["physics_step_s"])):
            plant.step(dynamics.gravity(plant.state().q))
        drift = float(np.max(np.abs(plant.state().q - q0)))
        if drift > 1e-8:
            raise AssertionError(f"Gravity equilibrium drifted {drift} rad")
        # Direction and torque units: M*qdd = tau-g, positive command => positive J7 acceleration.
        plant.reset()
        g = dynamics.gravity(plant.state().q)
        plant.data.ctrl[:] = g
        plant.data.ctrl[-1] += 0.01
        mujoco.mj_forward(plant.model, plant.data)
        acceleration = float(plant.data.qacc[-1])
        residual = float(np.max(np.abs(dynamics.mass(plant.state().q) @ plant.data.qacc -
                                       (plant.data.qfrc_actuator - g))))
        if acceleration <= 0 or residual > 1e-9:
            raise AssertionError("Joint torque mapping / dynamics residual failed")
        return {"status": "PASS", "samples": samples, "mujoco_version": mujoco.__version__,
                "pinocchio_version": other.pin.__version__ if other is not None else None,
                "urdf_sha256": hashlib.sha256(urdf_path(config).read_bytes()).hexdigest(),
                "config_sha256": hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
                "min_mass_eigenvalue_kg_m2": min_eigenvalue,
                "gravity_hold_drift_rad": drift, "joint7_acceleration_for_0p01_nm_rad_s2": acceleration,
                "forward_dynamics_residual_nm": residual,
                "cross_engine_max_abs_errors": errors if other is not None else "not run"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--pinocchio", action="store_true", help="Require Pinocchio, fail if missing")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = validate(load_config(args.config), compare_pinocchio=args.pinocchio)
    result = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result, encoding="utf-8")
    print(result, end="")


if __name__ == "__main__":
    main()
