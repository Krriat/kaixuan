"""Check the captured physical pose against launch offsets without touching hardware."""

import ast
import json
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class LeftCompensationTests(unittest.TestCase):
    def setUp(self):
        self.record = json.loads((ROOT / "config/left_arm_calibration.json").read_text())
        tree = ast.parse((ROOT / "launch/moveit_bringup.launch.py").read_text())
        self.defaults = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
                name = node.targets[0].id
                if name in {"_left_comp_defaults", "_comp_defaults", "_gear_ratio_defaults"}:
                    self.defaults[name] = list(map(float, ast.literal_eval(node.value)))

    def test_captured_encoder_positions_produce_screenshot_angles(self):
        self.assertEqual(len(self.record["joints"]), 7)
        for i, joint in enumerate(self.record["joints"]):
            with self.subTest(joint=joint["joint"]):
                comp = self.defaults["_left_comp_defaults"][i]
                gear = self.defaults["_gear_ratio_defaults"][i]
                self.assertEqual(comp, joint["new_comp_deg"])
                self.assertEqual(gear, joint["gear_ratio"])
                raw = joint["encoder_counts"] * 360 / (1048576 * gear)
                angle = math.radians(joint["direction"] * (raw - comp))
                self.assertAlmostEqual(angle, joint["target_rad"], delta=1e-9)
                self.assertLessEqual(abs(joint["target_rad"]), 3.14)

    def test_target_angle_maps_back_to_the_same_physical_position(self):
        for i, joint in enumerate(self.record["joints"]):
            with self.subTest(joint=joint["joint"]):
                comp = self.defaults["_left_comp_defaults"][i]
                degrees = joint["direction"] * math.degrees(joint["target_rad"]) + comp
                counts = round(degrees * 1048576 * joint["gear_ratio"] / 360)
                self.assertEqual(counts, joint["encoder_counts"])

    def test_right_arm_compensation_is_unchanged(self):
        self.assertEqual(self.defaults["_comp_defaults"], [
            8.61927750, -115.05901903, -151.79905785, 43.58607398,
            125.67501068, -9.17790101, -0.02567102,
        ])


if __name__ == "__main__":
    unittest.main()
