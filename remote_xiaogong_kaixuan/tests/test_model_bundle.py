"""Catch model-copy drift and preserve rejection of incompatible joint frames."""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from check_connection import model_difference_details


class ModelCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.client = (ROOT / "model/robot.urdf").read_text()

    def test_shoulder_translation_mismatch_names_the_field(self):
        robot = ET.fromstring(self.client)
        robot.find("joint[@name='right_joint1']/origin").set("xyz", "0.010173 -0.20995 -0.049457")
        differences = model_difference_details(ET.tostring(robot, encoding="unicode"), self.client)
        self.assertEqual(len(differences), 1)
        self.assertIn("right_joint1.origin.xyz", differences[0])
        self.assertIn("0.010173", differences[0])

    def test_xacro_wrapper_and_numeric_formatting_are_compatible(self):
        robot = ET.fromstring(self.client)
        robot.find("joint[@name='right_joint1']/origin").set("xyz", "0.0 -2.0995e-1 -0.0494570000")
        ET.SubElement(robot, "ros2_control", name="Hardware", type="system")
        self.assertEqual(model_difference_details(ET.tostring(robot, encoding="unicode"), self.client), [])

    def test_axis_limits_and_missing_joints_still_reject_the_model(self):
        robot = ET.fromstring(self.client)
        robot.find("joint[@name='right_joint2']/axis").set("xyz", "0 0 -1")
        robot.find("joint[@name='right_joint3']/limit").set("upper", "2.0")
        robot.remove(robot.find("joint[@name='left_joint7']"))
        differences = model_difference_details(ET.tostring(robot, encoding="unicode"), self.client)
        self.assertEqual(len(differences), 3)
        self.assertTrue(any("right_joint2.axis.xyz" in x for x in differences))
        self.assertTrue(any("right_joint3.limit.upper" in x for x in differences))
        self.assertTrue(any("left_joint7" in x for x in differences))

    def test_bundle_contains_every_referenced_mesh(self):
        for mesh in ET.fromstring(self.client).findall(".//mesh"):
            prefix = "package://xiaogong_kaixuan/meshes/"
            self.assertTrue(mesh.get("filename").startswith(prefix))
            self.assertTrue((ROOT / "model/meshes" / mesh.get("filename").removeprefix(prefix)).is_file())

    def test_bundle_matches_robot_source_when_both_are_in_the_checkout(self):
        source = ROOT.parent / "xiaogong_kaixuan/src/xiaogong_kaixuan"
        if not (source / "urdf/xiaogong-kaixuan.urdf").is_file():
            self.skipTest("standalone client bundle has no sibling robot workspace")
        self.assertEqual((source / "urdf/xiaogong-kaixuan.urdf").read_bytes(),
                         (ROOT / "model/robot.urdf").read_bytes(), "sync client model/ after changing robot URDF")
        for mesh in ET.fromstring(self.client).findall(".//mesh"):
            relative = mesh.get("filename").removeprefix("package://xiaogong_kaixuan/meshes/")
            self.assertEqual((source / "meshes" / relative).read_bytes(),
                             (ROOT / "model/meshes" / relative).read_bytes(), f"stale client mesh: {relative}")


if __name__ == "__main__":
    unittest.main()
