"""Offline kinematic checks for the left-arm zero pose; no ROS nodes or goals."""

import copy
import json
import math
from pathlib import Path
import random
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OFFSETS_DEG = [-180, -180, -7, 39, 180, -17, -6]
# Origins from the shoulder-adapted model in commit 2d714d5.
ORIGINAL_RPY = [
    [-5.551115123125783e-16, 1.3963, 1.570803673205103],
    [1.5708, -1.5708, 0], [1.5708, 0, 0],
    [0, -1.5708, math.pi], [0, 1.5708, 0],
    [0, -1.5708, 0], [0, 1.5708, 0],
]


def multiply(a, b):
    return [[sum(x * y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def transform(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]], [0, 0, 0, 1],
    ]


def frames(robot, angles):
    result = {"world": transform([0, 0, 0], [0, 0, 0])}
    for joint in robot.findall("joint"):
        origin = joint.find("origin")
        pose = transform(list(map(float, origin.get("xyz").split())),
                         list(map(float, origin.get("rpy").split())))
        if joint.get("type") == "revolute":
            # Every revolute axis in this model is joint-local +Z.
            assert joint.find("axis").get("xyz") == "0 0 1"
            pose = multiply(pose, transform([0, 0, 0], [0, 0, angles.get(joint.get("name"), 0)]))
        result[joint.find("child").get("link")] = multiply(result[joint.find("parent").get("link")], pose)
    return result


class ZeroCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.robot = ET.parse(ROOT / "model/robot.urdf").getroot()

    def test_zero_and_moving_poses_match_baseline_plus_offsets(self):
        baseline = copy.deepcopy(self.robot)
        for i, rpy in enumerate(ORIGINAL_RPY, 1):
            baseline.find(f"joint[@name='left_joint{i}']/origin").set("rpy", " ".join(map(str, rpy)))
        rng = random.Random(26)
        poses = [[0.0] * 7] + [[rng.uniform(-3.0, 3.0) for _ in range(7)] for _ in range(50)]
        for pose in poses:
            angles = {f"left_joint{i}": value for i, value in enumerate(pose, 1)}
            expected = frames(baseline, {name: value + math.radians(offset)
                                       for (name, value), offset in zip(angles.items(), OFFSETS_DEG)})
            actual = frames(self.robot, angles)
            for link in expected:
                with self.subTest(pose=pose, link=link):
                    for a, b in zip(sum(actual[link], []), sum(expected[link], [])):
                        self.assertAlmostEqual(a, b, delta=1e-9)

    def test_metadata_records_screenshot_and_source_hash(self):
        import hashlib
        metadata = json.loads((ROOT / "BUNDLE_INFO.json").read_text())
        self.assertEqual(metadata["left_zero_offsets_deg"], OFFSETS_DEG)
        source = ROOT.parent / metadata["source_model"]
        if source.is_file():
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), metadata["source_model_sha256"])

    def test_source_and_client_models_match_except_robot_name(self):
        source = ROOT.parent / "xiaogong_kaixuan/src/xiaogong_kaixuan/urdf/xiaogong-kaixuan.urdf"
        if not source.is_file():
            self.skipTest("standalone client has no robot source workspace")
        robot = ET.parse(source).getroot()
        robot.set("name", self.robot.get("name"))
        self.assertEqual(ET.tostring(robot), ET.tostring(self.robot))


if __name__ == "__main__":
    unittest.main()
