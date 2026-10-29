"""Run with ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1; synthetic feedback only."""
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/flight_recorder.py"
sys.path.insert(0, str(SCRIPT.parent))
spec = importlib.util.spec_from_file_location("flight_recorder", SCRIPT)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


def records(directory, stream):
    return [json.loads(line) for path in sorted(directory.glob(f"{stream}_*.jsonl"))
            for line in path.read_text().splitlines() if line.endswith("}")]


class WriterTests(unittest.TestCase):
    def test_nonfinite_and_legacy_fields(self):
        self.assertEqual(json.loads(recorder.json_line({"x": [math.nan, math.inf]})), {"x": [None, None]})
        msg = recorder.JointTrajectoryControllerState()
        msg.desired.positions = [1.0]
        self.assertEqual(list(recorder.point(msg, "reference", "desired").positions), [1.0])
        msg.reference.positions = [2.0]
        self.assertEqual(list(recorder.point(msg, "reference", "desired").positions), [2.0])
        self.assertIsNone(recorder.degrees_at([], 0))

    def test_rotation_sync_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "recording"
            writer = recorder.DurableWriter(path, segment_bytes=40, min_free_bytes=0)
            for i in range(20):
                writer.put("events", recorder.json_line({"sequence": i}))
            writer.put("joints", "row\n")
            writer.close()
            self.assertIsNone(writer.error)
            self.assertGreater(writer.last_sync_monotonic_ns, 0)
            self.assertGreater(len(list(path.glob("events_*.jsonl"))), 1)
            self.assertEqual([r["sequence"] for r in records(path, "events")], list(range(20)))
            self.assertEqual((path / "joints_0000.csv").read_text().splitlines()[0], ",".join(recorder.CSV_FIELDS))
            with self.assertRaises(FileExistsError):
                recorder.DurableWriter(path)

    def test_disk_space_failure_is_visible(self):
        with tempfile.TemporaryDirectory() as temporary:
            writer = recorder.DurableWriter(Path(temporary) / "recording", min_free_bytes=10**30)
            writer.thread.join(2)
            self.assertIn("free disk space", writer.error)
            writer.put("events", "{}\n")
            self.assertEqual(writer.dropped, 1)
            writer.close()


@unittest.skipUnless(os.environ.get("ROS_DOMAIN_ID") == "193" and os.environ.get("ROS_LOCALHOST_ONLY") == "1",
                     "requires explicitly isolated ROS domain 193 / localhost only")
class IntegrationTests(unittest.TestCase):
    def test_feedback_staleness_and_sigkill_durability(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
        rclpy.init()
        node = rclpy.create_node("flight_recorder_synthetic_test")
        js_pub = node.create_publisher(recorder.JointState, "/joint_states", qos_profile_sensor_data)
        cs_pub = node.create_publisher(recorder.JointTrajectoryControllerState,
                                       "/right_arm_controller/controller_state", qos_profile_sensor_data)
        hw_pub = node.create_publisher(recorder.DiagnosticArray,
                                       "/kaixuan/hardware_diagnostics", qos_profile_sensor_data)
        log_pub = node.create_publisher(recorder.Log, "/rosout", qos_profile_sensor_data)
        cycle_pub = node.create_publisher(recorder.Int64MultiArray, "/kaixuan/cycle_capture", 100)
        bus_pub = node.create_publisher(recorder.Int64MultiArray, "/kaixuan/bus_diagnostics", 100)
        from test_bus_diagnostics import packet as bus_packet
        from test_cycle_capture import batch
        from cycle_capture import decode, WIDTH, HEADER, AXIS
        process = None
        try:
            with tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary) / "recording"
                with open(Path(temporary) / "console.txt", "w+") as output:
                    process = subprocess.Popen([
                        sys.executable, str(SCRIPT), "--output-dir", str(directory),
                        "--active-arm", "right", "--real-hardware", "--expect-bus-diagnostics", "--config", str(SCRIPT),
                    ], stdout=output, stderr=subprocess.STDOUT)
                    deadline = time.monotonic() + 10
                    while js_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                        self.assertIsNone(process.poll())
                    self.assertGreater(js_pub.get_subscription_count(), 0)
                    start = time.monotonic()
                    count = 0
                    while time.monotonic() - start < 2.5:
                        msg = recorder.JointState()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        # Deliberately reverse names: verify mapping is by name, not index.
                        msg.name = ["right_joint2", "right_joint1"]
                        msg.position = [0.5, math.pi / 6]
                        msg.velocity = [0.0, 0.1]
                        js_pub.publish(msg)
                        cs = recorder.JointTrajectoryControllerState()
                        cs.header = msg.header
                        cs.joint_names = ["right_joint1"]
                        cs.desired.positions = [math.pi / 3]
                        cs.actual.positions = [math.pi / 6]
                        cs.error.positions = [math.pi / 6]
                        cs_pub.publish(cs)
                        hw = recorder.DiagnosticArray()
                        hw.header = msg.header
                        status = DiagnosticStatus(name="right_joint1", level=bytes([2]), message="synthetic drive fault")
                        status.values = [KeyValue(key="error_code", value="1234")]
                        hw.status = [status]
                        hw_pub.publish(hw)
                        if count % 10 == 0:
                            log_pub.publish(recorder.Log(name="synthetic_drive", level=40, msg="synthetic fault marker"))
                            bus_pub.publish(recorder.Int64MultiArray(data=bus_packet(
                                sequence=count // 10 + 1, raw=bytes([count // 10]) + bytes(19))))
                        if count % 3 == 0:
                            cycle_pub.publish(recorder.Int64MultiArray(data=batch((count // 3) * 50, fault=count == 30)))
                        count += 1
                        rclpy.spin_once(node, timeout_sec=0.02)
                    # Preserve old values with an explicit age when feedback stops.
                    until = time.monotonic() + 1.5
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertIsNone(process.poll())
                    process.kill()
                    process.wait(5)
                    with (directory / "joints_0000.csv").open() as csv_file:
                        rows = list(csv.DictReader(csv_file))
                    joint1 = [row for row in rows if row["joint"] == "right_joint1" and row["target_deg"]]
                    self.assertGreater(len(joint1), 8)
                    self.assertAlmostEqual(float(joint1[-1]["position_deg"]), 30.0)
                    self.assertAlmostEqual(float(joint1[-1]["target_deg"]), 60.0)
                    self.assertGreater(float(joint1[-1]["joint_state_age_s"]), 0.5)
                    self.assertTrue(all(row["position_deg"] == "" for row in rows if row["joint"].startswith("left")))
                    intervals = [(int(b["monotonic_ns"]) - int(a["monotonic_ns"])) / 1e9
                                 for a, b in zip(joint1, joint1[1:])]
                    self.assertAlmostEqual(sum(intervals) / len(intervals), 0.2, delta=0.05)
                    self.assertGreater(len(records(directory, "controllers")), 7)
                    self.assertLess(len(records(directory, "controllers")), 20)
                    self.assertTrue(any(r["data"]["status"][0]["values"][0]["value"] == "1234"
                                        for r in records(directory, "hardware")))
                    events = records(directory, "events")
                    self.assertTrue(any(r["kind"] == "config_snapshot" for r in events))
                    self.assertTrue(any(r["kind"] == "bus_probe_schema" for r in events))
                    self.assertTrue(any(r["kind"] == "esc_counter_change" for r in events))
                    bus_records = records(directory, "bus")
                    self.assertGreater(len(bus_records), 3)
                    self.assertEqual(bus_records[-1]['data']['joint'], 'right_joint2')
                    self.assertGreater(bus_records[-1]['data']['value']['port0_invalid_frames'], 0)
                    self.assertTrue(any(r["kind"] == "stream_health" and r["data"]["state"] == "stale" for r in events))
                    self.assertTrue(any(r["kind"] == "rosout" and r["data"]["msg"] == "synthetic fault marker" for r in events))
                    self.assertFalse(any(r["kind"] == "recorder_stopped" for r in events))
                    health = [r for r in events if r["kind"] == "recorder_health"]
                    self.assertTrue(all(r["data"]["dropped_records"] == 0 for r in health))
                    self.assertGreater(health[-1]["data"]["last_sync_monotonic_ns"], 0)
                    self.assertTrue(any(e['kind'] == 'drive_transition' and e['data']['values']['error_code'] == 400 for e in events))
                    waveform = records(directory, 'waveforms')
                    self.assertTrue(waveform)
                    recovered_errors = []
                    for block in waveform:
                        raw = decode(block['data'])
                        recovered_errors.extend(raw[o + len(HEADER) + len(AXIS) + 6] for o in range(0, len(raw), WIDTH))
                    self.assertIn(400, recovered_errors)

                    # New session must stop cleanly on Ctrl+C, without overwriting the old session.
                    next_directory = Path(temporary) / "next_recording"
                    process = subprocess.Popen([
                        sys.executable, str(SCRIPT), "--output-dir", str(next_directory),
                        "--active-arm", "right"], stdout=output, stderr=subprocess.STDOUT)
                    deadline = time.monotonic() + 10
                    while not (next_directory / "joints_0000.csv").exists() and time.monotonic() < deadline:
                        time.sleep(0.05)
                    process.send_signal(signal.SIGINT)
                    self.assertEqual(process.wait(5), 0)
                    self.assertTrue(any(r["kind"] == "recorder_stopped" for r in records(next_directory, "events")))
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main(verbosity=2)
