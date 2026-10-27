#!/usr/bin/env python3
"""Safe LinkerHand O6 monitor and opt-in small-motion test."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray, UInt16MultiArray
from std_srvs.srv import SetBool


class O6Test(Node):
    def __init__(self, namespace):
        super().__init__("linkerhand_o6_test")
        self.namespace = namespace.rstrip("/")
        self.actual = None
        self.status = None
        self.ready = None
        self.online = None
        self.create_subscription(
            UInt8MultiArray, f"{self.namespace}/actual_position", self._actual, 10)
        self.create_subscription(UInt16MultiArray, f"{self.namespace}/status", self._status, 10)
        self.create_subscription(Bool, f"{self.namespace}/ready", self._ready, 10)
        self.create_subscription(Bool, f"{self.namespace}/online", self._online, 10)
        self.publisher = self.create_publisher(
            UInt8MultiArray, f"{self.namespace}/target_position", 10)
        self.enable_client = self.create_client(SetBool, f"{self.namespace}/enable")

    def _actual(self, message):
        self.actual = list(message.data)

    def _status(self, message):
        self.status = list(message.data)

    def _ready(self, message):
        self.ready = message.data

    def _online(self, message):
        self.online = message.data

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def set_enabled(self, enabled):
        if not self.enable_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{self.namespace}/enable service is unavailable")
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_client.call_async(request)
        if not self.spin_until(future.done, 3.0):
            raise RuntimeError("enable service timed out")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(response.message if response else "enable service failed")
        print(response.message)

    def send(self, target):
        message = UInt8MultiArray()
        message.data = target
        self.publisher.publish(message)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Default mode only monitors O6 and never enables motion.")
    parser.add_argument(
        "--baseline", action="store_true",
        help="run the current-position enable handshake without intentional motion")
    parser.add_argument(
        "--exercise-index", action="store_true",
        help="after baseline, move index flexion -5 and restore it")
    parser.add_argument(
        "--disable-on-exit", action="store_true",
        help="explicitly disable O6 when this script exits")
    parser.add_argument(
        "--namespace", default="/linkerhand_o6",
        help="O6 ROS namespace; use /linkerhand_o6_left for the left hand")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = O6Test(args.namespace)
    enabled = False
    try:
        if not node.spin_until(
                lambda: node.actual is not None and node.status is not None and
                node.ready is not None and node.online is not None, 5.0):
            raise RuntimeError("no O6 status received")
        print(f"online={node.online} ready={node.ready} actual={node.actual}")
        print(f"status={node.status}")
        if not args.baseline and not args.exercise_index:
            print("monitor-only check complete; no enable or target command was sent")
            return 0

        original = list(node.actual)
        node.set_enabled(True)
        enabled = True
        if not node.spin_until(lambda: node.ready, 2.0):
            raise RuntimeError("O6 current-position handshake did not become ready")
        node.send(original)
        print(f"baseline ready at actual={node.actual}; same-position target sent")
        if not args.exercise_index:
            time.sleep(0.2)
            return 0

        moved = list(original)
        moved[2] = max(0, moved[2] - 5)
        if moved[2] == original[2]:
            raise RuntimeError("index flexion is already 0; cannot perform the -5 test")
        before_sequence = node.status[1]
        node.send(moved)
        if not node.spin_until(
                lambda: node.status is not None and node.status[1] != before_sequence, 1.0):
            raise RuntimeError("test target was not acknowledged")
        print(f"index target sent once: {original[2]} -> {moved[2]}")
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end:
            node.send(moved)
            rclpy.spin_once(node, timeout_sec=0.05)

        for restore_step in range(10):
            current = list(node.actual)
            restore = [
                max(value - 5, min(reference, value + 5))
                for value, reference in zip(current, original)
            ]
            restore_sequence = node.status[1]
            node.send(restore)
            if not node.spin_until(
                    lambda: node.status is not None and
                    node.status[1] != restore_sequence, 1.0):
                raise RuntimeError(
                    f"restore step {restore_step + 1} target {restore} was not acknowledged")
            print(f"restore step {restore_step + 1}: actual={current} target={restore}")

            end = time.monotonic() + 0.5
            while rclpy.ok() and time.monotonic() < end:
                node.send(restore)
                rclpy.spin_once(node, timeout_sec=0.05)
            if restore == original:
                print(f"restored original target={original}")
                return 0
        raise RuntimeError("restore did not reach the original target within 10 safe steps")
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"test failed: {error}", file=sys.stderr)
        return 1
    finally:
        if enabled and args.disable_on_exit and rclpy.ok():
            try:
                node.set_enabled(False)
            except RuntimeError as error:
                print(f"disable failed: {error}", file=sys.stderr)
        elif enabled:
            print("O6 remains enabled and holds the last PDO target")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
