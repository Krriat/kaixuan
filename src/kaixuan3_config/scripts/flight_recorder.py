#!/usr/bin/env python3
"""Crash-readable recorder; optional monitor-selector writes, never motion commands."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import queue
import shutil
import sys
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Log, ParameterEvent
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String
from std_msgs.msg import Int64MultiArray
from trajectory_msgs.msg import JointTrajectory
from cycle_capture import TriggeredCapture, HEADER as CYCLE_HEADER, AXIS as CYCLE_AXIS
from bus_diagnostics import BusEvidence, SCHEMA as BUS_SCHEMA
from monitor64 import Monitor64


JOINTS = [f"{arm}_joint{i}" for arm in ("right", "left") for i in range(1, 8)]
CSV_FIELDS = [
    "utc", "monotonic_ns", "joint", "active_arm", "position_deg", "velocity_deg_s",
    "target_deg", "controller_actual_deg", "error_deg", "target_velocity_deg_s",
    "joint_state_age_s", "controller_age_s", "joint_state_stamp_ns", "controller_stamp_ns",
]


def clean(value):
    """Strict JSON: unavailable/nonfinite numbers are null, never invented zeros."""
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def json_line(value):
    return json.dumps(clean(value), ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n"


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def point(message, preferred, legacy):
    # Empty reference/feedback fields exist in some Humble builds.
    candidate = getattr(message, preferred, None)
    return candidate if candidate is not None and len(candidate.positions) else getattr(message, legacy)


def degrees_at(values, index):
    return math.degrees(values[index]) if index < len(values) and math.isfinite(values[index]) else None


def sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


class DurableWriter:
    """One non-ROS thread owns all disk I/O; bounded queue exposes lost records."""

    def __init__(self, directory, sync_period=0.2, segment_bytes=256 * 1024**2,
                 min_free_bytes=1024**3):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        sync_directory(self.directory.parent)
        self.sync_period = sync_period
        self.segment_bytes = segment_bytes
        self.min_free_bytes = min_free_bytes
        self.pending = queue.Queue(maxsize=8192)
        self.dropped = 0
        self.error = None
        self.last_sync_monotonic_ns = 0
        self.last_sync_duration_ms = 0.0
        self.closed = threading.Event()
        self.files = {}
        self.thread = threading.Thread(target=self.run, name="diagnostic-disk-writer")
        self.thread.start()

    def put(self, stream, data):
        if self.error or self.closed.is_set():
            self.dropped += 1
            return
        try:
            self.pending.put_nowait((stream, data))
        except queue.Full:
            self.dropped += 1

    def write(self, stream, data):
        state = self.files.get(stream)
        if state is None or state[1] >= self.segment_bytes:
            number = 0 if state is None else state[2] + 1
            if state is not None:
                state[0].flush()
                os.fsync(state[0].fileno())
                state[0].close()
            suffix = "csv" if stream == "joints" else "jsonl"
            handle = open(self.directory / f"{stream}_{number:04d}.{suffix}", "x", encoding="utf-8")
            state = [handle, 0, number]
            self.files[stream] = state
            if stream == "joints":
                header = ",".join(CSV_FIELDS) + "\n"
                handle.write(header)
                state[1] += len(header)
            sync_directory(self.directory)
        state[0].write(data)
        state[1] += len(data.encode("utf-8"))

    def sync(self):
        start = time.monotonic()
        for handle, _, _ in self.files.values():
            handle.flush()
            os.fsync(handle.fileno())
        self.last_sync_monotonic_ns = time.monotonic_ns()
        self.last_sync_duration_ms = (time.monotonic() - start) * 1000

    def run(self):
        next_sync = time.monotonic() + self.sync_period
        next_space_check = 0.0
        try:
            while not self.closed.is_set() or not self.pending.empty():
                now = time.monotonic()
                if now >= next_space_check:
                    if shutil.disk_usage(self.directory).free < self.min_free_bytes:
                        raise OSError("Recording stopped: less than 1 GiB free disk space")
                    next_space_check = now + 1.0
                try:
                    stream, data = self.pending.get(timeout=max(0, next_sync - now))
                    self.write(stream, data)
                except queue.Empty:
                    pass
                if time.monotonic() >= next_sync:
                    self.sync()
                    next_sync = time.monotonic() + self.sync_period
            self.sync()
        except Exception as exc:
            self.error = str(exc)
            print(f"FLIGHT RECORDER FAILED: {exc}", file=sys.stderr, flush=True)
        finally:
            for handle, _, _ in self.files.values():
                try:
                    handle.close()
                except OSError:
                    pass

    def close(self):
        self.closed.set()
        self.thread.join(timeout=5)


class FlightRecorder(Node):
    def __init__(self, args):
        super().__init__("kaixuan_flight_recorder")
        self.args = args
        self.writer = DurableWriter(args.output_dir)
        self.monitor64 = None
        self.latest_joints = {}
        self.latest_controllers = {}
        self.last_seen = {}
        self.health_states = {}
        self.last_recorded = {}
        self.hardware_previous = {}
        self.capture = TriggeredCapture(self.record)
        self.bus_evidence = BusEvidence(self.record)
        self.subscriptions_kept = []
        self.record("events", "recorder_started", {
            "schema_version": 3, "argv": sys.argv, "pid": os.getpid(),
            "hostname": os.uname().nodename, "kernel": os.uname().release,
            "active_arm": args.active_arm, "sample_period_s": args.period,
            "launch_arguments": json.loads(args.launch_arguments),
            "environment": {key: os.environ.get(key) for key in (
                "ROS_DISTRO", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "ROS_LOG_DIR",
                "RMW_IMPLEMENTATION", "AMENT_PREFIX_PATH")},
        })
        self.record("events", "bus_probe_schema", BUS_SCHEMA)
        self.record("events", "cycle_schema", {
            "version": 1, "master_columns": CYCLE_HEADER, "axis_columns": CYCLE_AXIS,
            "axes_per_master": 7, "master0": "right", "master1": "left",
            "transition_flags": {"1": "CiA402/warning state change", "2": "control word change",
                                 "4": "error code change", "8": "position error crossed 1 degree"},
            "normal_period_s": args.period, "pre_seconds": 2, "post_seconds": 10,
        })
        for filename in args.config:
            try:
                data = Path(filename).read_bytes()
                self.record("events", "config_snapshot", {
                    "path": filename, "sha256": hashlib.sha256(data).hexdigest(),
                    "content": data.decode("utf-8", errors="replace"),
                })
            except OSError as exc:
                self.record("events", "config_snapshot_error", {"path": filename, "error": str(exc)})

        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscribe(JointState, "/joint_states", self.joints, qos)
        self.expected = ["/joint_states"]
        for arm in ("right", "left"):
            if args.active_arm != "both" and args.active_arm != arm:
                continue
            controller = f"/{arm}_arm_controller"
            topic = controller + "/controller_state"
            self.expected.append(topic)
            self.subscribe(JointTrajectoryControllerState, topic,
                           lambda msg, t=topic: self.controller(t, msg), qos)
            self.subscribe(GoalStatusArray, controller + "/follow_joint_trajectory/_action/status",
                           lambda msg, t=controller: self.record("events", t + "/action_status", message_to_ordereddict(msg)), latched)
            self.subscribe(JointTrajectory, controller + "/joint_trajectory",
                           lambda msg, t=controller: self.record("events", t + "/joint_trajectory", message_to_ordereddict(msg)), qos)
        if args.real_hardware:
            self.expected.append("/kaixuan/hardware_diagnostics")
            self.expected.append("/kaixuan/cycle_capture")
        if args.expect_bus_diagnostics:
            self.expected.append("/kaixuan/bus_diagnostics")
        self.subscribe(DiagnosticArray, "/kaixuan/hardware_diagnostics", self.hardware, qos)
        self.subscribe(Int64MultiArray, "/kaixuan/cycle_capture", self.cycles,
                       QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.subscribe(Int64MultiArray, "/kaixuan/bus_diagnostics", self.bus,
                       QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.subscribe(Log, "/rosout", self.rosout, qos)
        self.subscribe(ParameterEvent, "/parameter_events",
                       lambda msg: self.record("events", "parameter_event", message_to_ordereddict(msg)), qos)
        self.subscribe(String, "/robot_description",
                       lambda msg: self.record("events", "robot_description", {"xml": msg.data}), latched)
        self.clock_steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(args.period, self.sample, clock=self.clock_steady)
        self.create_timer(1.0, self.health, clock=self.clock_steady)
        self.get_logger().info(f"Flight recorder: {args.output_dir}; joint snapshots every {args.period}s")
        if args.real_hardware and args.expect_bus_diagnostics:
            self.record('monitor64', 'monitor64_started', {
                'selector': '0x2002:00 int16 = 64 (decimal)', 'value': '0x3000:00 int32',
                'baseline': '14 joints, >=1 second after transactions; right1/right2 prioritized when >=5s old',
                'trigger': 'position error crossing 1 degree or drive error; prioritize for 30 seconds',
                'restore': 'safe read failures retry per joint (1..10s); unconfirmed restore stops writes',
                'note': 'cumulative count; exact drive counting criterion unknown; no motion commands'})
            self.monitor64 = Monitor64(self.record)

    def subscribe(self, kind, topic, callback, qos):
        self.subscriptions_kept.append(self.create_subscription(kind, topic, callback, qos))

    def record(self, stream, kind, data):
        if stream == 'monitor64' and kind in ('monitor64_retry', 'monitor64_disabled',
                                              'restore_failed', 'shutdown_incomplete'):
            self.get_logger().error(f'Monitor64 {kind}: {data}')
        if kind == 'drive_transition' and self.monitor64 is not None:
            self.monitor64.trigger(data)
        self.writer.put(stream, json_line({
            "utc": utc_now(), "monotonic_ns": time.monotonic_ns(),
            "ros_time_ns": self.get_clock().now().nanoseconds, "kind": kind, "data": data,
        }))

    def joints(self, msg):
        now = time.monotonic()
        self.last_seen["/joint_states"] = now
        for i, name in enumerate(msg.name):
            self.latest_joints[name] = (
                degrees_at(msg.position, i), degrees_at(msg.velocity, i), now, stamp_ns(msg))

    def controller(self, topic, msg):
        now = time.monotonic()
        self.last_seen[topic] = now
        reference = point(msg, "reference", "desired")
        feedback = point(msg, "feedback", "actual")
        for i, name in enumerate(msg.joint_names):
            self.latest_controllers[name] = (
                degrees_at(reference.positions, i), degrees_at(feedback.positions, i),
                degrees_at(msg.error.positions, i), degrees_at(reference.velocities, i), now, stamp_ns(msg))
        if self.due(topic):
            self.record("controllers", topic, message_to_ordereddict(msg))

    def due(self, topic):
        now = time.monotonic()
        deadline = self.last_recorded.get(topic)
        if deadline is not None and now + 0.005 < deadline:
            return False
        next_deadline = deadline + self.args.period if deadline is not None else now + self.args.period
        self.last_recorded[topic] = next_deadline if next_deadline > now else now + self.args.period
        return True

    def cycles(self, msg):
        self.last_seen["/kaixuan/cycle_capture"] = time.monotonic()
        try:
            self.capture.receive(msg.data)
            if self.monitor64 is not None:
                self.monitor64.observe(msg.data)
        except ValueError as exc:
            self.record("events", "cycle_decode_error", {"error": str(exc)})
            self.get_logger().error(f"Cycle capture invalid: {exc}")

    def hardware(self, msg):
        self.last_seen["/kaixuan/hardware_diagnostics"] = time.monotonic()
        if not self.due("/kaixuan/hardware_diagnostics"):
            return
        data = message_to_ordereddict(msg)
        for status in data["status"]:
            if isinstance(status["level"], str):
                status["level"] = ord(status["level"])
            values = {v['key']: v['value'] for v in status['values']}
            if 'position_rad' in values:
                stamp, position = int(values['sample_steady_ns']), float(values['position_rad'])
                previous = self.hardware_previous.get(status['name'])
                if previous and stamp > previous[0]:
                    dt = (stamp - previous[0]) / 1e9
                    status['values'].extend([
                        {'key': 'position_delta_dt_s', 'value': str(dt)},
                        {'key': 'velocity_from_position_deg_s', 'value': str(math.degrees(position - previous[1]) / dt)},
                    ])
                self.hardware_previous[status['name']] = (stamp, position)
        self.record("hardware", "/kaixuan/hardware_diagnostics", data)

    def bus(self, msg):
        self.last_seen["/kaixuan/bus_diagnostics"] = time.monotonic()
        try:
            self.bus_evidence.receive(msg.data)
        except (ValueError, OverflowError) as exc:
            self.record("events", "bus_probe_decode_error", {"error": str(exc)})

    def rosout(self, msg):
        if msg.name != self.get_name():
            self.record("events", "rosout", message_to_ordereddict(msg))

    def sample(self):
        now = time.monotonic()
        utc = utc_now()
        mono = time.monotonic_ns()
        output = io.StringIO()
        writer = csv.writer(output)
        for name in JOINTS:
            js = self.latest_joints.get(name)
            cs = self.latest_controllers.get(name)
            enabled = self.args.active_arm == "both" or name.startswith(self.args.active_arm + "_")
            writer.writerow([
                utc, mono, name, int(enabled),
                js[0] if js and enabled else None, js[1] if js and enabled else None,
                *(cs[:4] if cs and enabled else [None] * 4),
                now - js[2] if js and enabled else None, now - cs[4] if cs and enabled else None,
                js[3] if js and enabled else None, cs[5] if cs and enabled else None,
            ])
        self.writer.put("joints", output.getvalue())

    def health(self):
        now = time.monotonic()
        monitor_health = self.monitor64.health() if self.monitor64 is not None else None
        if monitor_health:
            self.record('monitor64', 'monitor64_health', monitor_health)
            stale = [key for key, state in monitor_health['joints'].items()
                     if state['sample_age_s'] is None or state['sample_age_s'] > 60]
            if (monitor_health['blocked'] or not monitor_health['worker_alive'] or stale or
                    (monitor_health['age_s']>30 and not monitor_health['joints'])):
                if now-getattr(self, '_monitor_warning_time', 0)>10:
                    self._monitor_warning_time = now
                    self.get_logger().error(f'Monitor64 evidence incomplete: {monitor_health}')
        ages = {topic: now - self.last_seen[topic] if topic in self.last_seen else None
                for topic in self.expected}
        for topic, age in ages.items():
            limit = 2.5 if topic == "/kaixuan/bus_diagnostics" else 0.5
            state = "missing" if age is None else ("stale" if age > limit else "receiving")
            if self.health_states.get(topic) != state:
                self.record("events", "stream_health", {"topic": topic, "state": state, "age_s": age})
                self.health_states[topic] = state
                if state != "receiving":
                    self.get_logger().warning(f"Recorder input {state}: {topic}")
        self.record("events", "recorder_health", {
            "input_age_s": ages, "queue_depth": self.writer.pending.qsize(),
            "monitor64": monitor_health,
            "dropped_records": self.writer.dropped, "disk_error": self.writer.error,
            "last_sync_monotonic_ns": self.writer.last_sync_monotonic_ns,
            "last_sync_duration_ms": self.writer.last_sync_duration_ms,
            "load_average": os.getloadavg(),
            "cycle_gaps": self.capture.gaps, "cycle_ring_drops": self.capture.ring_drops,
            "bus_probe_gaps": self.bus_evidence.gaps, "bus_probe_ring_drops": self.bus_evidence.ring_drops,
        })
        if self.writer.error or self.writer.dropped:
            self.get_logger().error(
                f"Recorder incomplete: disk_error={self.writer.error}, dropped={self.writer.dropped}")

    def close(self):
        if self.monitor64 is not None:
            self.monitor64.close()
        for master, blocks in self.capture.history.items():
            for packet in blocks:
                self.capture.store(master, packet)
        self.record("events", "recorder_stopped", {"reason": "orderly shutdown"})
        self.writer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--period", type=float, default=0.2)
    parser.add_argument("--active-arm", choices=["both", "right", "left"], default="both")
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--expect-bus-diagnostics", action="store_true")
    parser.add_argument("--launch-arguments", default="{}")
    parser.add_argument("--config", action="append", default=[])
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.period) or args.period < 0.02:
        parser.error("period must be finite and >= 0.02 seconds")
    rclpy.init(args=ros_args)
    node = None
    try:
        node = FlightRecorder(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
