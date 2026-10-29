"""Regression tests for the environment passed to the client ROS processes."""

import os
import itertools
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(Path('/opt/ros/humble/setup.bash').is_file(), 'requires ROS 2 Humble setup')
class EnvironmentTests(unittest.TestCase):
    def run_shell(self, script, **overrides):
        environment = dict(os.environ)
        for name in ('ROS_DOMAIN_ID', 'KAIXUAN_ROS_DOMAIN_ID'):
            environment.pop(name, None)
        environment.update(overrides)
        return subprocess.run(
            ['bash', '-c', script, 'environment-test', str(ROOT / 'scripts/environment.sh'),
             str(ROOT / 'move_arm.sh')],
            env=environment, capture_output=True, text=True,
        )

    def source_environment(self, **overrides):
        return self.run_shell(
            'source "$1" || exit $?\nprintf "resolved=%s|%s|%s\\n" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$RMW_IMPLEMENTATION"',
            **overrides,
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
                self.assertIn('0～232', result.stdout + result.stderr)

    def option_combinations(self):
        for flags in itertools.product((False, True), repeat=3):
            setup = '\n'.join(f'set {"-" if enabled else "+"}o {name}'
                              for name, enabled in zip(('errexit', 'nounset', 'pipefail'), flags))
            yield flags, setup

    def test_success_preserves_all_shell_option_combinations(self):
        for flags, setup in self.option_combinations():
            with self.subTest(flags=flags):
                result = self.run_shell(setup + '\n' + '''
before="$SHELLOPTS"
client_root=caller_value
source "$1" || exit 7
[[ "$SHELLOPTS" == "$before" ]] || exit 8
[[ "$client_root" == caller_value ]] || exit 9
printf 'options-preserved\\n'
''')
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('options-preserved', result.stdout)

    def test_failure_preserves_options_and_returns_error(self):
        for flags, setup in self.option_combinations():
            with self.subTest(flags=flags):
                result = self.run_shell(setup + '\n' + '''
before="$SHELLOPTS"
if source "$1"; then exit 7; fi
[[ "$SHELLOPTS" == "$before" ]] || exit 8
printf 'failure-handled\\n'
''', ROS_DOMAIN_ID='invalid')
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('failure-handled', result.stdout)
                self.assertIn('0～232', result.stderr)

    def test_python_error_does_not_exit_the_calling_shell(self):
        result = self.run_shell('''
set +e
source "$1" || exit 7
python3 -c 'raise ValueError("out of joint limits")'
python_status=$?
printf 'python-status=%s; terminal-alive\\n' "$python_status"
[[ "$python_status" == 1 ]]
''')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('python-status=1; terminal-alive', result.stdout)
        self.assertIn('out of joint limits', result.stderr)

    def test_invalid_configuration_stops_wrapper_but_not_parent_shell(self):
        result = self.run_shell('''
set +e
bash "$2" --state
wrapper_status=$?
printf 'wrapper-status=%s; parent-alive\\n' "$wrapper_status"
[[ "$wrapper_status" != 0 ]]
''', ROS_DOMAIN_ID='invalid')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('wrapper-status=1; parent-alive', result.stdout)
        self.assertIn('0～232', result.stderr)


if __name__ == '__main__':
    unittest.main()
