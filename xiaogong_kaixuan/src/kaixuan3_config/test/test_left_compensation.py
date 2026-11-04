"""Check the captured physical pose against launch offsets without touching hardware."""

import ast
import importlib.util
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class LeftCompensationTests(unittest.TestCase):
    def setUp(self):
        self.record = json.loads((ROOT / "config/left_arm_calibration.json").read_text())
        self.launch_file = ROOT / "launch/moveit_bringup.launch.py"
        self.tree = ast.parse(self.launch_file.read_text())
        self.defaults = {}
        for node in ast.walk(self.tree):
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
                name = node.targets[0].id
                if name in {"_left_comp_defaults", "_comp_defaults", "_gear_ratio_defaults", "_left_direction_defaults"}:
                    self.defaults[name] = list(map(float, ast.literal_eval(node.value)))

    def hardware_parameters(self, overrides=None, use_launch=True):
        try:
            import xacro
            from launch import LaunchContext
            from launch.actions import DeclareLaunchArgument
            from launch.substitutions import LaunchConfiguration
        except ImportError:
            self.skipTest("requires the ROS 2 environment")

        mappings = dict(overrides or {})
        if use_launch:
            spec = importlib.util.spec_from_file_location("calibration_bringup", self.launch_file)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            description = module.generate_launch_description()
            context = LaunchContext()
            # Only declarations run: no ROS nodes, helpers or hardware are started.
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            context.launch_configurations.update(overrides or {})
            node = next(n for n in ast.walk(self.tree) if isinstance(n, ast.Assign)
                        and any(isinstance(t, ast.Name) and t.id == "real_mappings" for t in n.targets))
            substitutions = eval(
                compile(ast.Expression(node.value), str(self.launch_file), "eval"),
                {"__builtins__": {"range": range}, "LaunchConfiguration": LaunchConfiguration},
            )
            mappings = {name: value.perform(context) for name, value in substitutions.items()}

        path = ROOT.parent / "kaixuan_can_hardware/urdf/kaixuan1_ros2_control_real.urdf.xacro"
        robot = ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())
        return {p.get("name"): p.text for p in robot.findall("./ros2_control/hardware/param")}

    def test_captured_encoder_positions_produce_reference_angles(self):
        self.assertEqual(len(self.record["joints"]), 7)
        for i, joint in enumerate(self.record["joints"]):
            with self.subTest(joint=joint["joint"]):
                comp = self.defaults["_left_comp_defaults"][i]
                gear = self.defaults["_gear_ratio_defaults"][i]
                self.assertEqual(comp, joint["new_comp_deg"])
                self.assertEqual(gear, joint["gear_ratio"])
                self.assertEqual(self.defaults["_left_direction_defaults"][i], joint["direction"])
                raw = joint["encoder_counts"] * 360 / (1048576 * gear)
                angle = math.radians(joint["direction"] * (raw - comp))
                self.assertAlmostEqual(angle, joint["target_rad"], delta=1e-9)

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

    def test_direction_arguments_reach_the_real_hardware_xacro(self):
        expected = [1, 1, 1, 1, 1, 1, 1]
        parameters = self.hardware_parameters()
        direct_defaults = self.hardware_parameters(use_launch=False)
        for i, direction in enumerate(expected, start=1):
            self.assertEqual(int(parameters[f"left_direction{i}"]), direction)
            self.assertEqual(int(direct_defaults[f"left_direction{i}"]), direction)
            self.assertEqual(float(parameters[f"left_comp{i-1}"]),
                             self.defaults["_left_comp_defaults"][i-1])
        overridden = self.hardware_parameters({"left_direction2": "-1", "left_direction6": "-1"})
        self.assertEqual(int(overridden["left_direction2"]), -1)
        self.assertEqual(int(overridden["left_direction6"]), -1)

    def test_dc_clock_mode_reaches_the_real_hardware_xacro(self):
        for use_launch in (True, False):
            with self.subTest(use_launch=use_launch):
                defaults = self.hardware_parameters(use_launch=use_launch)
                self.assertEqual(defaults["dc_clock_mode"], "reference")
                self.assertEqual(defaults["cycle_time_ns"], "1000000")
                self.assertEqual(defaults["shift_time_ns"], "500000")
                self.assertEqual(defaults["active_arm"], "both")
                host = self.hardware_parameters({"dc_clock_mode": "host"}, use_launch=use_launch)
                self.assertEqual(host["dc_clock_mode"], "host")
                reference = self.hardware_parameters({"dc_clock_mode": "reference"}, use_launch=use_launch)
                self.assertEqual(reference["dc_clock_mode"], "reference")

    def test_direct_hardware_launch_forwards_dc_clock_mode(self):
        try:
            from launch import LaunchContext
            from launch.actions import DeclareLaunchArgument
            from launch.substitutions import Command, LaunchConfiguration
        except ImportError:
            self.skipTest("requires the ROS 2 environment")
        path = ROOT.parent / "kaixuan_can_hardware/launch/bringup.launch.py"
        spec = importlib.util.spec_from_file_location("direct_hardware_bringup", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        function = next(node for node in ast.parse(path.read_text()).body
                        if isinstance(node, ast.FunctionDef) and node.name == "generate_launch_description")
        for mode in ("host", "reference"):
            with self.subTest(mode=mode):
                context = LaunchContext()
                context.launch_configurations["dc_clock_mode"] = mode
                for action in description.entities:
                    if isinstance(action, DeclareLaunchArgument):
                        action.execute(context)
                values = {"__builtins__": {"range": range}, "LaunchConfiguration": LaunchConfiguration,
                          "robot_xacro": str(ROOT.parent / "kaixuan_can_hardware/urdf/kaixuan1_ros2_control_real.urdf.xacro")}
                for node in function.body:
                    if not isinstance(node, ast.Assign) or not isinstance(node.targets[0], ast.Name):
                        continue
                    name = node.targets[0].id
                    if name in {"controllers_yaml", "robot_xacro"}:
                        continue
                    values[name] = eval(compile(ast.Expression(node.value), str(path), "eval"), values)
                    if name == "xacro_command":
                        break
                # Command executes only the offline xacro converter, never ROS nodes.
                robot = ET.fromstring(Command(values["xacro_command"]).perform(context))
                hardware = {p.get("name"): p.text for p in robot.findall("./ros2_control/hardware/param")}
                self.assertEqual(hardware["dc_clock_mode"], mode)

    def test_both_urdfs_reverse_joint_axes_without_double_reversing_hardware(self):
        workspace = ROOT.parents[1]
        model = ET.parse(workspace / self.record["model_path"]).getroot()
        remote = ET.parse(workspace.parent / "remote_xiaogong_kaixuan/model/robot.urdf").getroot()
        parameters = self.hardware_parameters()
        for name, number in [("left_joint2", 2), ("left_joint6", 6)]:
            with self.subTest(joint=name):
                self.assertEqual(model.find(f"joint[@name='{name}']/axis").get("xyz"), "0 0 -1")
                self.assertEqual(remote.find(f"joint[@name='{name}']/axis").get("xyz"), "0 0 -1")
                self.assertEqual(int(parameters[f"left_direction{number}"]), 1)
        remote.set("name", model.get("name"))
        canonical = lambda element: ET.canonicalize(ET.tostring(element, encoding="unicode"), strip_text=True)
        self.assertEqual(canonical(model), canonical(remote))

    def test_feedback_and_command_signs_match_the_corrected_directions(self):
        parameters = self.hardware_parameters()
        for i, joint in enumerate(self.record["joints"], start=1):
            with self.subTest(joint=joint["joint"]):
                direction = int(parameters[f"left_direction{i}"])
                comp = float(parameters[f"left_comp{i-1}"])
                ratio = float(parameters[f"left_gear_ratio{i}"])
                counts_per_degree = 1048576 * ratio / 360
                feedback = lambda counts: direction * (counts / counts_per_degree - comp)
                feedback_delta = feedback(joint["encoder_counts"] + 1000) - feedback(joint["encoder_counts"])
                self.assertGreater(feedback_delta * direction, 0)
                counts = lambda angle: round((direction * math.degrees(angle) + comp) * counts_per_degree)
                command_delta = counts(joint["target_rad"] + 0.01) - counts(joint["target_rad"])
                self.assertGreater(command_delta * direction, 0)

    def test_normalized_reference_angles_are_within_the_current_urdf(self):
        model = ET.parse(ROOT.parents[1] / self.record["model_path"]).getroot()
        outside = []
        for joint in self.record["joints"]:
            limit = model.find(f"joint[@name='{joint['joint']}']/limit")
            if not float(limit.get("lower")) <= joint["target_rad"] <= float(limit.get("upper")):
                outside.append(joint["joint"])
        reported = [j["joint"] for j in self.record["reference_limit_violations"]]
        self.assertEqual(outside, reported)
        self.assertEqual(outside, [])
        self.assertTrue(self.record["validation"]["reference_angles_within_urdf_limits"])
        correction = next(j for j in self.record["mapping_adjustments"] if j["joint"] == "left_joint5")
        self.assertAlmostEqual(correction["old_value_deg"] - correction["new_value_deg"], 180)

    def test_joint5_normalization_preserves_pose_and_reaches_hardware(self):
        joint = next(j for j in self.record["joints"] if j["joint"] == "left_joint5")
        normalization = self.record["position_normalization"]
        self.assertAlmostEqual(joint["new_comp_deg"] - normalization["previous_comp_deg"], 360)
        self.assertEqual(joint["target_rad"], -math.pi)
        for counts in (joint["encoder_counts"] - 1000, joint["encoder_counts"],
                       joint["encoder_counts"] + 1000, 27824384):
            raw = counts * 360 / (1048576 * joint["gear_ratio"])
            previous = math.radians(joint["direction"] * (raw - normalization["previous_comp_deg"]))
            current = math.radians(joint["direction"] * (raw - joint["new_comp_deg"]))
            current = (current + math.pi) % (2 * math.pi) - math.pi
            self.assertGreaterEqual(current, -math.pi)
            self.assertLess(current, math.pi)
            self.assertAlmostEqual(math.sin(current), math.sin(previous), places=12)
            self.assertAlmostEqual(math.cos(current), math.cos(previous), places=12)
        import xacro
        model = ROOT.parent / "kaixuan_can_hardware/urdf/kaixuan1_ros2_control_real.urdf.xacro"
        robot = ET.fromstring(xacro.process_file(str(model)).toxml())
        params = {p.get("name"): p.text for p in robot.findall("./ros2_control/joint[@name='left_joint5']/param")}
        self.assertEqual(params["normalize_position"].lower(), "true")
        self.assertEqual(float(params["min_position_rad"]), -math.pi)
        self.assertEqual(float(params["max_position_rad"]), math.pi)


if __name__ == "__main__":
    unittest.main()
