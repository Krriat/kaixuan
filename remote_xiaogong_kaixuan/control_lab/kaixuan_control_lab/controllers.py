"""Only the stage-0 baselines are implemented; LQR/MPC/WBC are later exercises."""
import numpy as np


class GravityController:
    def reset(self, state):
        pass

    def compute(self, state, dynamics):
        return dynamics.gravity(state.q)


class ZeroTorqueController:
    def reset(self, state):
        pass

    def compute(self, state, dynamics):
        return np.zeros_like(state.q)
