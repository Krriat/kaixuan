"""MuJoCo plant and output-torque dynamics, with an optional actuator model."""
from collections import deque

import mujoco
import numpy as np

from .config import vector
from .interfaces import JointState


class Actuator:
    """clip -> optional slew -> integer delay -> exact first-order ZOH response."""

    def __init__(self, config):
        self.dt = config["physics_step_s"]
        self.limit = np.array(config["limits"]["torque_nm"], dtype=float)
        self.n = len(self.limit)
        settings = config["actuator"]
        self.delay_steps = settings["delay_steps"]
        self.time_constant = settings["time_constant_s"]
        self.slew = settings["torque_slew_nm_s"]
        self.reset()

    def reset(self):
        self.command = np.zeros(self.n)
        self.applied = np.zeros(self.n)
        self.queue = deque(np.zeros(self.n) for _ in range(self.delay_steps))

    def step(self, requested):
        requested = vector(requested, self.n, "requested torque")
        target = np.clip(requested, -self.limit, self.limit)
        if self.slew is not None:
            delta = np.asarray(self.slew) * self.dt
            target = self.command + np.clip(target - self.command, -delta, delta)
        self.command = target.copy()
        self.queue.append(target.copy())
        delayed = self.queue.popleft()
        alpha = 1.0 if self.time_constant == 0 else -np.expm1(-self.dt / self.time_constant)
        self.applied += alpha * (delayed - self.applied)
        return self.applied.copy()


class MujocoDynamics:
    """Separate scratch data: evaluating a controller model never steps the plant."""

    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)

    def _evaluate(self, q, dq=None):
        self.data.qpos[:] = vector(q, self.model.nq, "q")
        self.data.qvel[:] = 0 if dq is None else vector(dq, self.model.nv, "dq")
        mujoco.mj_forward(self.model, self.data)

    def mass(self, q):
        self._evaluate(q)
        result = np.empty((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, result, self.data.qM)  # MuJoCo 3.3.7 API
        return result

    def gravity(self, q):
        self._evaluate(q)
        return self.data.qfrc_bias.copy()  # dq=0 => gravity only

    def bias(self, q, dq):
        self._evaluate(q, dq)
        return self.data.qfrc_bias.copy()  # c(q,dq) + g(q), excludes passive friction

    def kinematics(self, q):
        self._evaluate(q)
        site_id = self.model.site("ee_site").id
        linear = np.empty((3, self.model.nv))
        angular = np.empty((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, linear, angular, site_id)
        return (self.data.site_xpos[site_id].copy(),
                self.data.site_xmat[site_id].reshape(3, 3).copy(),
                np.vstack((linear, angular)))


class MujocoPlant:
    def __init__(self, path, config):
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(str(path))
        n = len(config["joint_names"])
        if (self.model.nq, self.model.nv, self.model.nu) != (n, n, n):
            raise ValueError("Model dimensions disagree with configuration")
        for i, name in enumerate(config["joint_names"]):
            if self.model.joint(name).id != i or self.model.actuator(f"{name}_torque").id != i:
                raise ValueError("Model joint / actuator order mismatch")
        if not np.isclose(self.model.opt.timestep, config["physics_step_s"], rtol=0, atol=1e-12):
            raise ValueError("Physics timestep mismatch")
        self.data = mujoco.MjData(self.model)
        self.dynamics = MujocoDynamics(self.model)
        self.actuator = Actuator(config)
        self.reset()

    def reset(self, q=None, dq=None):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = vector(self.config["initial_position_rad"] if q is None else q,
                                   self.model.nq, "q")
        self.data.qvel[:] = 0 if dq is None else vector(dq, self.model.nv, "dq")
        self.actuator.reset()
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def state(self):
        return JointState(float(self.data.time), self.data.qpos.copy(), self.data.qvel.copy())

    def check_state(self):
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError("Non-finite simulated state")
        if np.any(np.abs(self.data.qvel) > np.asarray(self.config["limits"]["velocity_rad_s"])):
            raise RuntimeError("Simulation velocity guard exceeded (90 deg/s by default)")
        lower, upper = self.model.jnt_range.T
        if np.any(self.data.qpos < lower) or np.any(self.data.qpos > upper):
            raise RuntimeError("Simulation position guard exceeded")

    def step(self, requested):
        self.check_state()
        warnings_before = self.data.warning.number.copy()
        self.data.ctrl[:] = self.actuator.step(requested)
        mujoco.mj_step(self.model, self.data)
        if np.any(self.data.warning.number > warnings_before):
            raise RuntimeError("MuJoCo warning during integration; inspect experiment parameters")
        self.check_state()
        return self.state()
