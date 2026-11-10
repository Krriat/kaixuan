#!/usr/bin/env python3
"""只读左臂 ROS 反馈，打印弧度、角度和可复制的 TARGET_JOINTS_DEG。"""

import argparse
import math
import os
import time

os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

JOINTS = [f"left_joint{i}" for i in range(1, 8)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", action="store_true", help="持续刷新，Ctrl+C 退出")
    parser.add_argument("--interval", type=float, default=0.5, help="刷新间隔，秒")
    parser.add_argument("--timeout", type=float, default=5.0, help="反馈超时，秒")
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.interval, args.timeout)):
        parser.error("--interval and --timeout must be finite and positive")

    rclpy.init(args=[])
    node = Node("read_left_joint_angles")
    latest = None
    received_at = time.monotonic()

    def on_state(message):
        nonlocal latest, received_at
        if len(message.name) != len(message.position):
            return
        positions = dict(zip(message.name, message.position))
        if all(name in positions and math.isfinite(positions[name]) for name in JOINTS):
            latest = [float(positions[name]) for name in JOINTS]
            received_at = time.monotonic()

    node.create_subscription(JointState, "/joint_states", on_state, qos_profile_sensor_data)
    next_print = 0.0
    result = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - received_at > args.timeout:
                print("未收到新鲜的左臂七轴 /joint_states；确认 bringup 已启动，ROS_DOMAIN_ID 与启动终端一致。", flush=True)
                result = 1
                break
            if latest is None or now < next_print:
                continue
            degrees = [math.degrees(value) for value in latest]
            print(f"\n/joint_states | ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}")
            print("joint           rad          deg")
            for name, radians, angle in zip(JOINTS, latest, degrees):
                print(f"{name:<13} {radians:>10.6f} {angle:>12.6f}")
            print("TARGET_JOINTS_DEG = [" + ", ".join(f"{value:.6f}" for value in degrees) + "]", flush=True)
            if not args.watch:
                break
            next_print = now + args.interval
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
