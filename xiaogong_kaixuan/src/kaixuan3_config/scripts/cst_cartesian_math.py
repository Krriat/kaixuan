"""Robot-side model, reference checks and bounded torque feedback (no ROS writes)."""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

# The existing Orin Pinocchio installation is private; do not replace ROS's Python.
if importlib.util.find_spec("pinocchio") is None:
    candidate = (Path.home() / ".local/share/kaixuan/pinocchio/opt/ros/humble/lib"
                 / f"python{sys.version_info.major}.{sys.version_info.minor}/site-packages")
    if candidate.is_dir():
        sys.path.insert(0, str(candidate))
import pinocchio as pin

# Reuse only numerical planning code. No MuJoCo, ROS transport or desktop paths.
for ancestor in Path(__file__).resolve().parents:
    lab = ancestor / "remote_xiaogong_kaixuan/control_lab"
    if (lab / "kaixuan_control_lab/trajectory.py").is_file():
        sys.path.insert(0, str(lab))
        break
else:
    raise ImportError("Keep remote_xiaogong_kaixuan/control_lab beside the robot workspace")
from kaixuan_control_lab.trajectory import rotation_error
from cst_cartesian_trajectory import build_timed_reference

JOINTS = [f"left_joint{i}" for i in range(1, 8)]


def validate_resume_pose(snapshot, q, dq):
    """Accept only a stationary, disabled saved pose matching fresh encoders."""
    expected = np.deg2rad([snapshot["q_deg"][j] for j in JOINTS])
    saved_speed = np.array([snapshot["velocity_deg_s"][j] for j in JOINTS])
    if not np.isfinite(np.r_[expected, saved_speed, q, dq]).all():
        raise ValueError("Resume pose contains nonfinite feedback")
    for joint in JOINTS:
        d = snapshot["diagnostics"][joint]
        if (int(d["control_word"]) not in (0, 6) or (int(d["status_word"]) & 0x6f) not in (0x21, 0x40)
                or any(int(d[k]) for k in ("effort_claimed", "torque_armed", "error_code", "torque_guard_fault"))
                or int(d["target_torque_raw"]) != 0):
            raise ValueError(f"Resume snapshot is not a clean disabled stop: {joint}")
    if np.max(np.abs(saved_speed)) > .1 or np.max(np.abs(dq)) > np.deg2rad(.1):
        raise ValueError("Resume requires stationary saved and current feedback")
    mismatch = np.rad2deg(q-expected)
    if np.max(np.abs(mismatch)) > .25:
        raise ValueError(f"Current pose does not match saved stop within 0.25 deg: {mismatch.tolist()}")
    return mismatch


def vector(config, key, positive=False):
    value = np.asarray(config[key], dtype=float)
    if value.shape != (7,) or not np.isfinite(value).all() or (positive and np.any(value <= 0)):
        raise ValueError(f"{key} must contain seven finite {'positive ' if positive else ''}values")
    return value


def hardware_settings(xml):
    root = ET.fromstring(xml)
    system = root.find("ros2_control")
    if system is None:
        raise ValueError("Active robot_description has no ros2_control system")
    hardware = system.find("hardware")
    params = {item.get("name"): item.text for item in hardware.findall("param")}
    expected = {"control_mode": "cst", "active_arm": "left", "stop_cst_group_on_fault": "true"}
    if hardware.findtext("plugin") != "kaixuan_can_hardware/KaixuanCanSystem":
        raise ValueError("This client requires real Kaixuan hardware")
    if any(params.get(k) != v for k, v in expected.items()):
        raise ValueError(f"Use cst_cartesian_test.launch.py; required hardware settings: {expected}")
    if params.get("cst_joints", "").split(",") != JOINTS:
        raise ValueError("CST joint selection/order must be left_joint1 through left_joint7")
    joints = {j.get("name"): j for j in system.findall("joint")}
    if any([x.get("name") for x in joints[j].findall("command_interface")] != ["effort"] for j in JOINTS):
        raise ValueError("The seven left joints must expose only effort commands")
    limits = {}
    for key in ("effort_limit_nm", "effort_slew_nm_s", "max_velocity_rad_s",
                "max_displacement_rad", "min_position_rad", "max_position_rad"):
        limits[key] = np.array([float(joints[j].find(f"param[@name='{key}']").text) for j in JOINTS])
    limits["effort_release_slew_nm_s"] = np.array([
        float(joints[j].findtext("param[@name='effort_release_slew_nm_s']", default="0"))
        for j in JOINTS])
    release = limits["effort_release_slew_nm_s"]
    increase = limits["effort_slew_nm_s"]
    if not np.isfinite(release).all() or np.any((release != 0.) & (release < increase)):
        raise ValueError("Invalid release slew: use zero or a value >= increase slew")
    limits["effort_release_slew_nm_s"] = np.where(release == 0., increase, release)
    return params, limits


def slew_torque(previous, desired, dt, increase, release):
    """Match the hardware magnitude limiter, including time spent crossing zero."""
    crossing = previous*desired < 0.
    to_zero = np.abs(previous)/release
    rate = np.where(np.abs(desired) < np.abs(previous), release, increase)
    ordinary = previous+np.clip(desired-previous, -rate*dt, rate*dt)
    toward_zero = previous-np.sign(previous)*release*dt
    after_zero = np.sign(desired)*np.minimum(np.abs(desired), increase*np.maximum(dt-to_zero, 0.))
    return np.where(crossing, np.where(to_zero < dt, after_zero, toward_zero), ordinary)


class RobotModel:
    def __init__(self, xml, config, right_position):
        self.full = pin.buildModelFromXML(xml)
        self.full_q = pin.neutral(self.full)
        for i, angle in enumerate(right_position, 1):
            self.full_q[self.full.joints[self.full.getJointId(f"right_joint{i}")].idx_q] = angle
        locked = [i for i in range(1, self.full.njoints) if self.full.names[i] not in JOINTS]
        self.model = pin.buildReducedModel(self.full, locked, self.full_q)
        if list(self.model.names)[1:] != JOINTS or self.model.nq != 7:
            raise ValueError("Pinocchio joint order mismatch")
        self.model.gravity.linear = np.array([0., 0., -9.80665])
        self.data = self.model.createData()
        self.extra_inertia = vector(config, "armature_kg_m2")
        if np.any(self.extra_inertia < 0):
            raise ValueError("Negative armature")
        self.frame_id = self.model.getFrameId(config["end_effector_frame"], pin.FrameType.BODY)
        if self.frame_id >= self.model.nframes:
            raise ValueError("End-effector BODY frame missing")
        root = ET.fromstring(xml)
        parents = {j.find("child").get("link") for j in root.findall("joint")}
        roots = {l.get("name") for l in root.findall("link")} - parents
        if roots != {"world"}:
            raise ValueError(f"Expected URDF root world, got {roots}")

    def mass(self, q):
        upper = np.triu(pin.crba(self.model, self.data, q))
        return upper + np.triu(upper, 1).T + np.diag(self.extra_inertia)

    def bias(self, q, dq):
        return pin.nonLinearEffects(self.model, self.data, q, dq).copy()

    def inverse_dynamics(self, q, dq, ddq):
        return pin.rnea(self.model, self.data, q, dq, ddq).copy() + self.extra_inertia * ddq

    def kinematics(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self.frame_id]
        jac = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id,
                                      pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return placement.translation.copy(), placement.rotation.copy(), jac.copy()

    def check_collisions(self, urdf_file, package_dirs, srdf_file, samples):
        geometry = pin.buildGeomFromUrdf(self.full, str(urdf_file), pin.GeometryType.COLLISION,
                                         package_dirs=package_dirs)
        geometry.addAllCollisionPairs()
        pin.removeCollisionPairs(self.full, geometry, str(srdf_file))
        active_ids = {self.full.getJointId(j) for j in JOINTS}
        indices = [self.full.joints[self.full.getJointId(j)].idx_q for j in JOINTS]
        omitted_pairs = []
        omitted_right = 0
        for pair in list(geometry.collisionPairs):
            objects = [geometry.geometryObjects[i] for i in (pair.first, pair.second)]
            # Only the left arm is calibrated. A fictitious right-arm FK pose
            # must not be advertised as a measurement or a collision check.
            has_unknown_right = any(self.full.names[obj.parentJoint].startswith("right_") for obj in objects)
            if has_unknown_right or not any(obj.parentJoint in active_ids for obj in objects):
                # Store values, not Boost.Python references into a vector that
                # removeCollisionPair will compact and invalidate.
                omitted_pairs.append((pair.first, pair.second))
                omitted_right += int(has_unknown_right)
        for first, second in omitted_pairs:
            geometry.removeCollisionPair(pin.CollisionPair(first, second))
        gd, data = pin.GeometryData(geometry), self.full.createData()
        q = self.full_q.copy()
        for i, left in enumerate(samples):
            q[indices] = left
            if pin.computeCollisions(self.full, data, geometry, gd, q, True):
                pairs = [(geometry.geometryObjects[p.first].name, geometry.geometryObjects[p.second].name)
                         for p, r in zip(geometry.collisionPairs, gd.collisionResults) if r.isCollision()]
                raise ValueError(f"Self collision at reference sample {i}: {pairs}")
        return {"samples": len(samples), "pairs": len(geometry.collisionPairs),
                "omitted_uncalibrated_right_arm_pairs": omitted_right,
                "scope": "left arm versus itself/body only, with SRDF exclusions; right arm and external environment NOT checked"}


def joint_reference(start, end, t, duration):
    s = np.clip(t / duration, 0., 1.)
    progress = s**3 * (10. - 15.*s + 6.*s*s)
    velocity = 30.*s*s*(1.-s)**2 / duration
    acceleration = 60.*s*(1.-s)*(1.-2.*s) / duration**2
    delta = end - start
    return start + progress*delta, velocity*delta, acceleration*delta


def make_plan(model, q_initial, config, limits):
    target = np.deg2rad(vector(config, "target_joints_deg"))
    lower = np.maximum(model.model.lowerPositionLimit, limits["min_position_rad"])
    upper = np.minimum(model.model.upperPositionLimit, limits["max_position_rad"])
    displacement = np.asarray(config["displacement_world_m"], dtype=float)
    if displacement.shape != (3,) or not np.isfinite(displacement).all():
        raise ValueError("Invalid Cartesian displacement")
    settings = {"control_period_s": .02, "trajectory": {
        "start_hold_s": 1., "move_s": config["line_s"], "end_hold_s": 1.,
        "displacement_world_m": displacement, "ik_tolerance": 1e-10, "max_ik_iterations": 40},
        "acceptance": {"reference_position_m": 1e-6, "reference_orientation_rad": 1e-6}}
    settings["trajectory"].update(profile=config.get("line_profile", "quintic"),
                                  cruise_ramp_s=config.get("cruise_ramp_s", 1.))
    reference = build_timed_reference(model, target, lower, upper, settings, .01)
    approach_t = np.linspace(0., config["approach_s"], round(config["approach_s"] / .01)+1)
    approach = [joint_reference(q_initial, target, t, config["approach_s"]) for t in approach_t]
    samples = np.vstack(([v[0] for v in approach], reference.q))
    speeds = np.vstack(([v[1] for v in approach], reference.dq))
    tau_approach = np.array([model.inverse_dynamics(*v) for v in approach])
    peaks = np.max(np.abs(np.vstack((tau_approach, reference.tau))), axis=0)
    if np.any(samples <= lower) or np.any(samples >= upper):
        raise ValueError("Reference outside position limits")
    if np.any(np.max(np.abs(samples-q_initial), axis=0) >= limits["max_displacement_rad"] - np.deg2rad(5)):
        raise ValueError("Reference too close to displacement-from-enable guard")
    if np.any(np.max(np.abs(speeds), axis=0) > np.minimum(
            limits["max_velocity_rad_s"], np.deg2rad(config["max_reference_speed_deg_s"]))):
        raise ValueError("Reference too fast; increase approach_s/line_s")
    if np.any(peaks > .8 * limits["effort_limit_nm"]):
        raise ValueError("Insufficient nominal torque margin")
    return reference, samples, {"initial_joints_deg": np.rad2deg(q_initial).tolist(),
        "target_joints_deg": np.rad2deg(target).tolist(), "final_joints_deg": np.rad2deg(reference.q[-1]).tolist(),
        "start_ee_world_m": reference.start_position.tolist(),
        "end_ee_world_m": (reference.start_position+reference.displacement).tolist(),
        "peak_reference_speed_deg_s": np.rad2deg(np.max(np.abs(speeds), axis=0)).tolist(),
        "peak_nominal_torque_nm": peaks.tolist(), "line": reference.diagnostics}


class TorqueFeedback:
    """Inverse dynamics plus torque-domain PID; saturation/slew anti-windup."""
    def __init__(self, model, config, limits):
        self.model = model
        self.kp = vector(config, "kp_nm_per_rad", True)
        self.kd = vector(config, "kd_nm_s_per_rad", True)
        self.ki = vector(config, "ki_nm_per_rad_s", True)
        self.integral_limit = vector(config, "integral_limit_nm", True)
        self.limit, self.slew = limits["effort_limit_nm"], limits["effort_slew_nm_s"]
        self.integral, self.previous = np.zeros(7), np.zeros(7)

    def select(self, phase, elapsed):
        """PID has no trajectory-dependent gain schedule."""

    def step(self, q, dq, reference, dt, scale=1.):
        qr, dqr, ddqr = reference
        error = qr-q
        self.feedforward = self.model.inverse_dynamics(q, dq, ddqr)
        self.state_feedback = self.kp*error + self.kd*(dqr-dq)
        base = self.feedforward + self.state_feedback
        candidate_i = np.clip(self.integral + self.ki*error*dt, -self.integral_limit, self.integral_limit)
        if scale < 1.:
            candidate_i = self.integral.copy()
        raw = scale*(base+candidate_i)
        command = np.clip(np.clip(raw, -self.limit, self.limit),
                          self.previous-self.slew*dt, self.previous+self.slew*dt)
        # Freeze an integral component when it would push further into either limit.
        integrate = (raw-command)*error <= 0.
        self.integral = np.where(integrate, candidate_i, self.integral)
        raw = scale*(base+self.integral)
        command = np.clip(np.clip(raw, -self.limit, self.limit),
                          self.previous-self.slew*dt, self.previous+self.slew*dt)
        self.previous = command.copy()
        return command, raw, self.integral.copy()
