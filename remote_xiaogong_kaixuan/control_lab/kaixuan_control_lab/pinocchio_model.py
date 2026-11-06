"""Optional independent model; no hardcoded Orin / ROS installation path.

The diagonal armature is added explicitly, exactly once, to M and RNEA.
Pinocchio's raw URDF model has no knowledge of control_lab/config/left_arm.yaml.
"""
import numpy as np

from .config import armature, urdf_path, vector


class PinocchioDynamics:
    def __init__(self, config):
        import pinocchio as pin
        self.pin = pin
        full = pin.buildModelFromUrdf(str(urdf_path(config)))
        active = config["joint_names"]
        locked = [i for i in range(1, full.njoints) if full.names[i] not in active]
        self.model = pin.buildReducedModel(full, locked, pin.neutral(full))
        self.model.gravity.linear = np.asarray(config["gravity_m_s2"], dtype=float)
        if list(self.model.names)[1:] != active or self.model.nq != len(active):
            raise ValueError("Pinocchio joint order disagrees with controller coordinates")
        self.data = self.model.createData()
        self.extra_inertia = armature(config)
        self.frame_id = self.model.getFrameId(config["end_effector_frame"], pin.FrameType.BODY)
        if self.frame_id >= self.model.nframes:
            raise ValueError("End-effector BODY frame missing from Pinocchio model")

    def mass(self, q):
        q = vector(q, self.model.nq, "q")
        upper = np.triu(self.pin.crba(self.model, self.data, q))
        return upper + np.triu(upper, 1).T + np.diag(self.extra_inertia)

    def gravity(self, q):
        return self.pin.computeGeneralizedGravity(self.model, self.data,
                                                  vector(q, self.model.nq, "q")).copy()

    def bias(self, q, dq):
        return self.pin.nonLinearEffects(self.model, self.data,
                                        vector(q, self.model.nq, "q"),
                                        vector(dq, self.model.nv, "dq")).copy()

    def inverse_dynamics(self, q, dq, ddq):
        ddq = vector(ddq, self.model.nv, "ddq")
        return self.pin.rnea(self.model, self.data, vector(q, self.model.nq, "q"),
                             vector(dq, self.model.nv, "dq"), ddq).copy() + self.extra_inertia * ddq

    def kinematics(self, q):
        q = vector(q, self.model.nq, "q")
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self.frame_id]
        jacobian = self.pin.computeFrameJacobian(self.model, self.data, q, self.frame_id,
                                                self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        # EigenPy squeezes a 6x1 matrix to (6,) for a single active joint.
        # Preserve the interface's 6 x nv shape, avoiding NumPy broadcasting.
        jacobian = np.asarray(jacobian).reshape(6, self.model.nv)
        # Same ordering as mj_jacSite: linear then angular, at the tool origin,
        # components expressed in world axes (NOT Pinocchio's WORLD convention).
        return placement.translation.copy(), placement.rotation.copy(), jacobian.copy()
