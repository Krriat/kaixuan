import copy
from pathlib import Path
import tempfile
import unittest

import numpy as np

from kaixuan_control_lab.backend import Actuator, MujocoPlant
from kaixuan_control_lab.config import load_config
from kaixuan_control_lab.model import build_model
from kaixuan_control_lab.validate import validate


class FoundationTests(unittest.TestCase):
    def test_gravity_equilibrium_mass_and_torque_direction(self):
        self.assertEqual(validate(load_config())["status"], "PASS")

    def test_armature_changes_only_diagonal_mass_not_gravity(self):
        config = load_config()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.xml"
            plants = []
            for multiplier in (0.5, 1.0, 2.0):
                modified = copy.deepcopy(config)
                modified["inertia"]["armature_kg_m2"]["left_joint7"] = multiplier * 0.02
                plants.append(MujocoPlant(build_model(modified, path, visuals=False), modified))
            q = np.array([0.3, -0.4, 0.2, 0.1, -0.2, 0.5, -0.3])
            base = plants[0].dynamics
            for plant, delta in zip(plants[1:], (0.01, 0.03)):
                expected = np.zeros((7, 7))
                expected[-1, -1] = delta
                np.testing.assert_allclose(plant.dynamics.mass(q) - base.mass(q), expected, atol=1e-12)
                np.testing.assert_allclose(plant.dynamics.gravity(q), base.gravity(q), atol=1e-12)

    def test_output_torque_is_not_multiplied_by_gear_ratio(self):
        config = load_config()
        with tempfile.TemporaryDirectory() as directory:
            plant = MujocoPlant(build_model(config, Path(directory) / "model.xml", visuals=False), config)
            np.testing.assert_allclose(plant.model.actuator_gear[:, 0], np.ones(7))
            np.testing.assert_allclose(plant.model.dof_armature, [0, 0, 0, 0, 0, 0, 0.02])
            self.assertEqual((plant.model.nq, plant.model.nu), (7, 7))

    def test_clip_slew_delay_and_first_order_response(self):
        config = load_config()
        actuator = Actuator(config)
        np.testing.assert_allclose(actuator.step(np.full(7, 100.0)), config["limits"]["torque_nm"])
        config["actuator"]["delay_steps"] = 2
        config["actuator"]["time_constant_s"] = config["physics_step_s"] / np.log(2)
        actuator = Actuator(config)
        np.testing.assert_allclose(actuator.step(np.ones(7)), 0)
        np.testing.assert_allclose(actuator.step(np.ones(7)), 0)
        np.testing.assert_allclose(actuator.step(np.ones(7)), 0.5)
        np.testing.assert_allclose(actuator.step(np.ones(7)), 0.75)
        config["actuator"].update(delay_steps=0, time_constant_s=0,
                                  torque_slew_nm_s=config["limits"]["hardware_torque_slew_nm_s"])
        actuator = Actuator(config)
        expected = np.array(config["limits"]["hardware_torque_slew_nm_s"]) * config["physics_step_s"]
        np.testing.assert_allclose(actuator.step(np.ones(7)), expected)
        actuator.reset()
        np.testing.assert_allclose(actuator.step(-np.ones(7)), -expected)

    def test_rejects_nonfinite_or_wrong_size_commands(self):
        actuator = Actuator(load_config())
        for request in (np.zeros(6), np.full(7, float("nan")), np.full(7, float("inf"))):
            with self.assertRaises(ValueError):
                actuator.step(request)

    def test_guard_catches_velocity_and_position(self):
        config = load_config()
        with tempfile.TemporaryDirectory() as directory:
            plant = MujocoPlant(build_model(config, Path(directory) / "model.xml", visuals=False), config)
            plant.data.qvel[-1] = np.deg2rad(90.1)
            with self.assertRaisesRegex(RuntimeError, "velocity guard"):
                plant.step(np.zeros(7))
            plant.reset()
            plant.data.qpos[-1] = 4.0
            with self.assertRaisesRegex(RuntimeError, "position guard"):
                plant.step(np.zeros(7))


if __name__ == "__main__":
    unittest.main()
