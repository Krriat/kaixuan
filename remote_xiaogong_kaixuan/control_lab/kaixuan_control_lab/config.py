"""Portable configuration shared by the simulation and dynamics model."""
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/left_arm.yaml"


def vector(value, size, label, *, nonnegative=False):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label}: expected {size} finite numbers")
    if nonnegative and np.any(result < 0):
        raise ValueError(f"{label}: negative values are invalid")
    return result


def load_config(path=DEFAULT_CONFIG):
    with Path(path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if config["schema_version"] != 1:
        raise ValueError("Unsupported configuration schema")
    names = config["joint_names"]
    if not names or len(names) != len(set(names)):
        raise ValueError("joint_names must be nonempty and unique")
    n = len(names)
    dt = config["physics_step_s"]
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("physics_step_s must be finite and positive")
    vector(config["initial_position_rad"], n, "initial_position_rad")
    vector(config["gravity_m_s2"], 3, "gravity_m_s2")
    for key in ("torque_nm", "velocity_rad_s", "hardware_torque_slew_nm_s"):
        values = vector(config["limits"][key], n, key, nonnegative=True)
        if np.any(values == 0):
            raise ValueError(f"{key} must be positive")
    for key in ("viscous_nm_s_rad", "coulomb_nm"):
        vector(config["physical_friction"][key], n, key, nonnegative=True)
    for name, value in config["inertia"]["armature_kg_m2"].items():
        if not np.isfinite(value) or value < 0:
            raise ValueError(f"Invalid armature for {name}")
    actuator = config["actuator"]
    if type(actuator["delay_steps"]) is not int or actuator["delay_steps"] < 0:
        raise ValueError("delay_steps must be a nonnegative integer")
    if not np.isfinite(actuator["time_constant_s"]) or actuator["time_constant_s"] < 0:
        raise ValueError("time_constant_s must be finite and nonnegative")
    if actuator["torque_slew_nm_s"] is not None:
        vector(actuator["torque_slew_nm_s"], n, "torque_slew_nm_s", nonnegative=True)
    if config["contact_enabled"]:
        raise ValueError("Contact geometry has not been validated; use the free-space baseline")
    return config


def urdf_path(config):
    return (ROOT / config["urdf"]).resolve()


def armature(config):
    values = config["inertia"]["armature_kg_m2"]
    return np.array([values.get(name, 0.0) for name in config["joint_names"]])
