"""Controller boundary: SI joint state in, TOTAL output-side torque out."""
from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class JointState:
    time_s: float
    q: np.ndarray
    dq: np.ndarray


class Dynamics(Protocol):
    def mass(self, q: np.ndarray) -> np.ndarray: ...
    def gravity(self, q: np.ndarray) -> np.ndarray: ...
    def bias(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray: ...


class TorqueController(Protocol):
    def reset(self, state: JointState) -> None: ...
    def compute(self, state: JointState, dynamics: Dynamics) -> np.ndarray: ...
