"""Regression tests for the environment passed to the client ROS processes."""

import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(Path('/opt/ros/humble/setup.bash').is_file(), 'requires ROS 2 Humble setup')
class EnvironmentTests(unittest.TestCase):
    def source_environment(self, **overrides):
        environment = dict(os.environ)
        for name in ('ROS_DOMAIN_ID', 'KAIXUAN_ROS_DOMAIN_ID'):
            environment.pop(name, None)
        environment.update(overrides)
        return subprocess.run(
            ['bash', '-c', 'source "$1"\nprintf "resolved=%s|%s|%s\\n" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$RMW_IMPLEMENTATION"',
             'environment-test', str(ROOT / 'scripts/environment.sh')],
            env=environment, capture_output=True, text=True,
        )

    def assert_domain(self, expected, **overrides):
        result = self.source_environment(**overrides)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f'resolved={expected}|0|rmw_fastrtps_cpp', result.stdout)

    def test_default_matches_robot_domain_26(self):
        self.assert_domain(26)

    def test_exported_standard_variable_is_not_replaced_by_project_default(self):
        self.assert_domain(26, ROS_DOMAIN_ID='26', KAIXUAN_ROS_DOMAIN_ID='0')

    def test_explicit_zero_remains_valid(self):
        self.assert_domain(0, ROS_DOMAIN_ID='0', KAIXUAN_ROS_DOMAIN_ID='26')

    def test_other_standard_domain_is_preserved(self):
        self.assert_domain(193, ROS_DOMAIN_ID='193')

    def test_project_override_when_standard_variable_is_missing(self):
        self.assert_domain(187, KAIXUAN_ROS_DOMAIN_ID='187')

    def test_invalid_domain_is_rejected_before_starting_ros(self):
        for value in ('-1', '233', 'abc', '9999999999999999999999999999999'):
            with self.subTest(value=value):
                result = self.source_environment(ROS_DOMAIN_ID=value)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('0～232', result.stdout)


if __name__ == '__main__':
    unittest.main()
