"""Registration and loading helpers for custom KGU BAM parameter files."""

from __future__ import annotations

from pathlib import Path

from bam.actuators import actuators
from bam.model import load_model
from bam.testbench import Pendulum

from .actuator import KguActuator


PACKAGE_DIR = Path(__file__).resolve().parent
PARAMS_DIR = PACKAGE_DIR / "params"


def register_kgu_actuator() -> None:
    """Register the custom actuator before BAM parses a KGU JSON file."""

    actuators.setdefault("kaixuan_kgu", lambda: KguActuator(Pendulum))


def load_kgu_model(name: str):
    """Load one of the bundled bootstrap models by directory name."""

    register_kgu_actuator()
    path = PARAMS_DIR / name / "bootstrap_m1.json"
    if not path.is_file():
        available = sorted(item.name for item in PARAMS_DIR.iterdir() if item.is_dir())
        raise FileNotFoundError(f"Unknown KGU model {name!r}; available: {available}")
    return load_model(str(path))
