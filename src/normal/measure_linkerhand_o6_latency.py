#!/usr/bin/env python3
"""Move one LinkerHand O6 joint by a small step and measure response latency."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray, UInt16MultiArray
from std_srvs.srv import SetBool


JOINT_NAMES = [
    "拇指弯曲",
    "拇指横摆",
    "食指弯曲",
    "中指弯曲",
    "无名指弯曲",
    "小指弯曲",
]
APPLIED_SEQUENCE_INDEX = 1
COMMAND_SEQUENCE_INDEX = 3


class O6LatencyTest(Node):
    def __init__(self, namespace):
        super().__init__("linkerhand_o6_latency_test")
        self.namespace = namespace.rstrip("/")
        self.actual = None
        self.actual_stamp_ns = 0
        self.status = None
        self.status_stamp_ns = 0
        self.ready = None
        self.online = None

        self.create_subscription(
            UInt8MultiArray,
            f"{self.namespace}/actual_position",
            self._on_actual,
            10,
        )
        self.create_subscription(
            UInt16MultiArray, f"{self.namespace}/status", self._on_status, 10
        )
        self.create_subscription(Bool, f"{self.namespace}/ready", self._on_ready, 10)
        self.create_subscription(Bool, f"{self.namespace}/online", self._on_online, 10)
        self.target_publisher = self.create_publisher(
            UInt8MultiArray, f"{self.namespace}/target_position", 10
        )
        self.enable_client = self.create_client(SetBool, f"{self.namespace}/enable")

    def _on_actual(self, message):
        self.actual = list(message.data)
        self.actual_stamp_ns = time.monotonic_ns()

    def _on_status(self, message):
        self.status = list(message.data)
        self.status_stamp_ns = time.monotonic_ns()

    def _on_ready(self, message):
        self.ready = message.data

    def _on_online(self, message):
        self.online = message.data

    def spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def set_enabled(self, enabled):
        if not self.enable_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{self.namespace}/enable 服务不可用")
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_client.call_async(request)
        if not self.spin_until(future.done, 3.0):
            raise RuntimeError("使能服务调用超时")
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else "无服务响应"
            raise RuntimeError(f"使能失败：{message}")
        print(response.message)

    def publish_target(self, target):
        message = UInt8MultiArray()
        message.data = target
        send_time_ns = time.monotonic_ns()
        self.target_publisher.publish(message)
        return send_time_ns

    def measure_motion(self, target, joint, tolerance, timeout_sec, label):
        if self.actual is None or self.status is None or len(self.status) <= COMMAND_SEQUENCE_INDEX:
            raise RuntimeError("O6 状态不完整")

        start_position = list(self.actual)
        previous_command_sequence = self.status[COMMAND_SEQUENCE_INDEX]
        send_time_ns = self.publish_target(target)
        applied_time_ns = None
        response_time_ns = None
        reached_time_ns = None
        observed_command_sequence = None
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.status is not None and len(self.status) > COMMAND_SEQUENCE_INDEX:
                applied_sequence = self.status[APPLIED_SEQUENCE_INDEX]
                command_sequence = self.status[COMMAND_SEQUENCE_INDEX]
                if command_sequence != previous_command_sequence:
                    observed_command_sequence = command_sequence
                    if applied_sequence == command_sequence and applied_time_ns is None:
                        applied_time_ns = self.status_stamp_ns

            if self.actual is not None and self.actual_stamp_ns >= send_time_ns:
                if self.actual[joint] != start_position[joint] and response_time_ns is None:
                    response_time_ns = self.actual_stamp_ns
                if abs(self.actual[joint] - target[joint]) <= tolerance:
                    if reached_time_ns is None:
                        reached_time_ns = self.actual_stamp_ns

            if (
                applied_time_ns is not None
                and response_time_ns is not None
                and reached_time_ns is not None
            ):
                break

        print(f"\n{label}：{JOINT_NAMES[joint]} {start_position[joint]} -> {target[joint]}")
        if observed_command_sequence is not None:
            print(f"  CommandSequence: {previous_command_sequence} -> {observed_command_sequence}")
        else:
            print("  CommandSequence: 未观察到变化")
        self._print_latency("指令传输延迟", send_time_ns, applied_time_ns)
        self._print_latency("实际响应延迟", send_time_ns, response_time_ns)
        self._print_latency("到位时间", send_time_ns, reached_time_ns)
        print(f"  当前反馈位置: {self.actual}")

        missing = []
        if applied_time_ns is None:
            missing.append("AppliedCommandSequence 未追上 CommandSequence")
        if response_time_ns is None:
            missing.append("ActualPosition 未发生变化")
        if reached_time_ns is None:
            missing.append("未在容差内到位")
        if missing:
            raise RuntimeError(f"{label}超时：" + "；".join(missing))

    @staticmethod
    def _print_latency(name, start_ns, end_ns):
        if end_ns is None:
            print(f"  {name}: 超时")
        else:
            print(f"  {name}: {(end_ns - start_ns) / 1_000_000.0:.3f} ms")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "自动小幅移动 LinkerHand O6 的一个关节，并测量指令确认、首次动作和到位耗时。"
        )
    )
    parser.add_argument(
        "--joint",
        type=int,
        choices=range(6),
        default=2,
        metavar="0..5",
        help="关节编号，默认2（食指弯曲）",
    )
    parser.add_argument(
        "--delta", type=int, default=-100, help="相对当前位置的测试步长，默认-5"
    )
    parser.add_argument(
        "--tolerance", type=int, default=1, help="到位允许误差，默认1"
    )
    parser.add_argument(
        "--timeout", type=float, default=5.0, help="每次动作超时秒数，默认5"
    )
    parser.add_argument(
        "--hold", type=float, default=0.5, help="到达测试位置后的停留秒数，默认0.5"
    )
    parser.add_argument(
        "--no-return", action="store_true", help="测试后不自动返回原位置"
    )
    parser.add_argument(
        "--keep-enabled",
        action="store_true",
        help="如果脚本执行了使能，退出时仍保持使能",
    )
    parser.add_argument(
        "--namespace", default="/linkerhand_o6",
        help="O6 ROS namespace; use /linkerhand_o6_left for the left hand",
    )
    return parser.parse_args()


def make_small_target(actual, joint, requested_delta):
    target = list(actual)
    target[joint] = max(0, min(255, target[joint] + requested_delta))
    if target[joint] == actual[joint]:
        target[joint] = max(0, min(255, actual[joint] - requested_delta))
    if target[joint] == actual[joint]:
        raise RuntimeError("测试步长为0，或者当前位置无法按指定步长移动")
    return target


def main():
    args = parse_args()
    if args.delta == 0:
        print("参数错误：--delta 不能为0", file=sys.stderr)
        return 2
    if args.tolerance < 0:
        print("参数错误：--tolerance 不能小于0", file=sys.stderr)
        return 2
    if args.timeout <= 0 or args.hold < 0:
        print("参数错误：--timeout 必须大于0，--hold 不能小于0", file=sys.stderr)
        return 2

    rclpy.init()
    node = O6LatencyTest(args.namespace)
    enabled_by_script = False
    original = None
    try:
        if not node.spin_until(
            lambda: (
                node.actual is not None
                and len(node.actual) == 6
                and node.status is not None
                and len(node.status) > COMMAND_SEQUENCE_INDEX
                and node.ready is not None
                and node.online is not None
            ),
            5.0,
        ):
            raise RuntimeError("5秒内未收到完整 O6 状态")
        if not node.online:
            raise RuntimeError("O6 当前不在线")

        original = list(node.actual)
        print(f"在线状态正常，当前位置: {original}")
        print("提示：当前 ROS 状态发布周期为10 ms，测量结果具有约10 ms量化误差。")

        if not node.ready:
            node.set_enabled(True)
            enabled_by_script = True
            if not node.spin_until(lambda: node.ready is True, 3.0):
                raise RuntimeError("当前位置使能握手未进入 ready=true")
        else:
            print("O6 已经处于 ready=true，保持原有使能状态。")

        target = make_small_target(original, args.joint, args.delta)
        print(
            f"即将自动测试 {JOINT_NAMES[args.joint]}："
            f"{original[args.joint]} -> {target[args.joint]}"
        )
        node.spin_for(1.0)
        node.measure_motion(
            target, args.joint, args.tolerance, args.timeout, "测试动作"
        )

        if not args.no_return:
            node.spin_for(args.hold)
            node.measure_motion(
                original, args.joint, args.tolerance, args.timeout, "返回动作"
            )
        else:
            print("已按 --no-return 要求停留在测试目标。")
        return 0
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"测试失败：{error}", file=sys.stderr)
        return 1
    finally:
        if enabled_by_script and not args.keep_enabled and rclpy.ok():
            try:
                node.set_enabled(False)
                print("脚本已将 O6 失能。")
            except RuntimeError as error:
                print(f"退出时失能失败：{error}", file=sys.stderr)
        elif enabled_by_script:
            print("O6 按 --keep-enabled 要求保持使能。")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
