#!/usr/bin/env python3
"""Read-only fresh, stationary, disabled left-arm snapshot for --resume-from."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout < 3:
        parser.error("--timeout must be at least 3 seconds")
    if args.output.exists():
        parser.error("Use a new output path; saved snapshots are not overwritten")
    joints = [f"left_joint{i}" for i in range(1, 8)]
    samples, diagnostics = deque(), {}
    rclpy.init()
    node = rclpy.create_node("cst_standby_snapshot")
    def state(message):
        if not all(j in message.name for j in joints):
            return
        indices = [message.name.index(j) for j in joints]
        if any(i >= len(message.position) or i >= len(message.velocity) for i in indices):
            return
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if not -.01 <= node.get_clock().now().nanoseconds*1e-9-stamp <= .05:
            return
        now = time.monotonic()
        q = [math.degrees(message.position[i]) for i in indices]
        v = [math.degrees(message.velocity[i]) for i in indices]
        samples.append((now, q, v))
        while samples and now-samples[0][0] > 3.1:
            samples.popleft()
    def diagnostic(message):
        diagnostics.update({s.name: {v.key: v.value for v in s.values}
                            for s in message.status if s.name in joints})
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(JointState, "/joint_states", state, qos)
    node.create_subscription(DiagnosticArray, "/kaixuan/hardware_diagnostics", diagnostic, qos)
    deadline = time.monotonic()+args.timeout
    reason = "waiting for fresh joint state and diagnostics"
    def require(condition, message):
        if not condition:
            raise ValueError(message)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if not samples or set(diagnostics) != set(joints):
                continue
            try:
                require(now-samples[-1][0] < .05, "joint feedback stale")
                require(samples[-1][0]-samples[0][0] >= 3., "waiting for 3 seconds of stationary feedback")
                for j in joints:
                    d = diagnostics[j]
                    require(0 <= now-int(d["sample_steady_ns"])*1e-9 < .7, f"{j}: diagnostic snapshot stale")
                    require(int(d["actual_mode"]) == 10, f"{j}: not CST")
                    require(int(d["control_word"]) in (0, 6), f"{j}: control word is not disabled")
                    require(int(d["status_word"]) & 0x6f in (0x40, 0x21), f"{j}: drive is not disabled")
                    for key in ["effort_claimed", "torque_armed", "torque_guard_fault", "error_code", "target_torque_raw"]:
                        require(int(d[key]) == 0, f"{j}: {key} is nonzero; deactivate torque_controller before sampling")
                require(all(math.isfinite(x) for _, q, v in samples for x in q+v), "nonfinite feedback")
                require(max(abs(x) for _, _, v in samples for x in v) <= .1, "arm is still moving")
                span = [max(q[i] for _, q, _ in samples)-min(q[i] for _, q, _ in samples) for i in range(7)]
                require(max(span) <= .05, "arm has not settled within 0.05 degrees")
            except (KeyError, ValueError) as error:
                reason = str(error)
                continue
            result = {"q_deg": dict(zip(joints, samples[-1][1])),
                      "velocity_deg_s": dict(zip(joints, samples[-1][2])),
                      "diagnostics": diagnostics, "observed_position_span_deg": span,
                      "feedback_age_s": now-samples[-1][0], "read_only": True}
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump(result, stream, indent=2, allow_nan=False)
                stream.write("\n")
            print(json.dumps({"saved": str(args.output.resolve()), "q_deg": result["q_deg"]}, indent=2))
            return 0
        raise RuntimeError(f"No standby snapshot saved: {reason}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
