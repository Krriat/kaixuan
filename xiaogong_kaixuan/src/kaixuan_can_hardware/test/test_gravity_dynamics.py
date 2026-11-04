"""Compare the real URDF against independent KDL dynamics and potential energy."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import PyKDL as kdl

if importlib.util.find_spec("pinocchio") is None:
    sys.path.insert(0, str(Path.home() / ".local/share/kaixuan/pinocchio/opt/ros/humble/lib/python3.10/site-packages"))
import pinocchio as pin

URDF = Path(__file__).resolve().parents[2] / "xiaogong_kaixuan/urdf/xiaogong-kaixuan.urdf"


def xyz(text):
    return [float(v) for v in text.split()]


def kdl_chain(robot, tip):
    links = {link.get("name"): link for link in robot.findall("link")}
    by_child = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}
    ordered = []
    child = tip
    while child != "base_link":
        joint = by_child[child]
        ordered.append(joint)
        child = joint.find("parent").get("link")
    chain = kdl.Chain()
    for joint in reversed(ordered):
        origin = joint.find("origin")
        frame = kdl.Frame(kdl.Rotation.RPY(*xyz(origin.get("rpy", "0 0 0"))),
                          kdl.Vector(*xyz(origin.get("xyz", "0 0 0"))))
        if joint.get("type") == "fixed":
            k_joint = kdl.Joint(joint.get("name"), kdl.Joint.Fixed)
        else:
            axis = frame.M * kdl.Vector(*xyz(joint.find("axis").get("xyz")))
            k_joint = kdl.Joint(joint.get("name"), frame.p, axis, kdl.Joint.RotAxis)
        child = joint.find("child").get("link")
        inertial = links[child].find("inertial")
        if inertial is None:
            inertia = kdl.RigidBodyInertia()
        else:
            mass = float(inertial.find("mass").get("value"))
            com = kdl.Vector(*xyz(inertial.find("origin").get("xyz", "0 0 0")))
            # Only mass and COM affect static gravity; rotational inertia is unused.
            inertia = kdl.RigidBodyInertia(mass, com, kdl.RotationalInertia())
        chain.addSegment(kdl.Segment(child, k_joint, frame, inertia))
    return chain


class GravityDynamicsTests(unittest.TestCase):
    def test_real_urdf_gravity_and_potential_gradient(self):
        robot = ET.parse(URDF).getroot()
        model = pin.buildModelFromUrdf(str(URDF))
        model.gravity.linear = np.array([0.0, 0.0, -9.80665])
        data = model.createData()
        rng = np.random.default_rng(2411)
        max_difference = max_gradient_error = 0.0
        for arm in ("left", "right"):
            chain = kdl_chain(robot, f"{arm}_ee")
            solver = kdl.ChainDynParam(chain, kdl.Vector(0, 0, -9.80665))
            self.assertEqual(chain.getNrOfJoints(), 7)
            indices = [model.joints[model.getJointId(f"{arm}_joint{i}")].idx_q for i in range(1, 8)]
            for sample in range(25):
                angles = np.array([0, 0, 0, 0, -math.pi if arm == "left" else 0, 0, 0], dtype=float) if sample == 0 else rng.uniform(-2.5, 2.5, 7)
                q = np.zeros(model.nq)
                q[indices] = angles
                expected = pin.computeGeneralizedGravity(model, data, q).copy()[indices]
                kq, kg = kdl.JntArray(7), kdl.JntArray(7)
                for i in range(7):
                    kq[i] = angles[i]
                self.assertEqual(solver.JntToGravity(kq, kg), 0)
                actual = np.array([kg[i] for i in range(7)])
                max_difference = max(max_difference, np.max(np.abs(actual - expected)))
                np.testing.assert_allclose(actual, expected, atol=1e-8, rtol=1e-8)
                for i, index in enumerate(indices):
                    step = np.zeros(model.nq)
                    step[index] = 1e-6
                    gradient = (pin.computePotentialEnergy(model, data, q + step) - pin.computePotentialEnergy(model, data, q - step)) / 2e-6
                    max_gradient_error = max(max_gradient_error, abs(gradient - expected[i]))
                    self.assertAlmostEqual(gradient, expected[i], delta=1e-6)
        print(f"Pinocchio/KDL max error: {max_difference:.3g} Nm; potential gradient max error: {max_gradient_error:.3g} Nm")


if __name__ == "__main__":
    unittest.main()
