#!/usr/bin/env python3

import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class ServoZOscillator(Node):
    def __init__(self, topic: str, frame_id: str) -> None:
        super().__init__("servo_z_oscillator")
        self._frame_id = frame_id
        self._publisher = self.create_publisher(
            TwistStamped,
            topic,
            qos_profile_sensor_data,
        )

    def publish_z(self, value: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.twist.linear.z = value
        self._publisher.publish(message)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Alternate MoveIt Servo Cartesian Z commands"
    )
    parser.add_argument("--amplitude", type=float, default=0.2)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument(
        "--cycles",
        type=int,
        default=5,
        help="Number of -Z/+Z round trips; use 0 to run until Ctrl+C",
    )
    parser.add_argument("--topic", default="/servo_demo_node/delta_twist_cmds")
    parser.add_argument("--frame-id", default="base_link")
    args = parser.parse_args()

    if not 0.0 < args.amplitude <= 1.0:
        parser.error("--amplitude must be in (0, 1]")
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.cycles < 0:
        parser.error("--cycles must be zero or positive")
    return args


def publish_for(
    node: ServoZOscillator,
    value: float,
    duration: float,
    rate: float,
) -> None:
    period = 1.0 / rate
    count = max(1, round(duration * rate))
    deadline = time.monotonic()
    for _ in range(count):
        if not rclpy.ok():
            return
        node.publish_z(value)
        rclpy.spin_once(node, timeout_sec=0.0)
        deadline += period
        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)


def publish_stop(node: ServoZOscillator, rate: float) -> None:
    for _ in range(10):
        node.publish_z(0.0)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / rate)


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ServoZOscillator(args.topic, args.frame_id)
    cycle = 0

    print(
        f"topic={args.topic} frame={args.frame_id} amplitude=±{args.amplitude} "
        f"duration={args.duration}s rate={args.rate}Hz cycles={args.cycles or 'infinite'}"
    )
    print("Starting with -Z. Press Ctrl+C to stop.")

    try:
        while rclpy.ok() and (args.cycles == 0 or cycle < args.cycles):
            cycle += 1
            label = f"{cycle}/{args.cycles}" if args.cycles else str(cycle)
            print(f"cycle {label}: z={-args.amplitude:+.3f}", flush=True)
            publish_for(node, -args.amplitude, args.duration, args.rate)
            if not rclpy.ok():
                break
            print(f"cycle {label}: z={args.amplitude:+.3f}", flush=True)
            publish_for(node, args.amplitude, args.duration, args.rate)
    except KeyboardInterrupt:
        print("\nInterrupted; sending stop commands...", flush=True)
    finally:
        if rclpy.ok():
            publish_stop(node, args.rate)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
