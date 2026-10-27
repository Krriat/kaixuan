#!/usr/bin/env python3
"""Continuously stream O6 position targets while pressure scanning stays enabled."""

import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
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
CAN_TX_COUNT_INDEX = 6
CAN_RX_COUNT_INDEX = 7
PRESSURE_FINGER_MASK = 0x1F
PRESSURE_COMPLETE_STATE = 2
PRESSURE_MAX_AGE_MS = 500


def counter_delta(previous, current):
    """Return the forward delta of a wrapping uint16 counter."""
    return (current - previous) & 0xFFFF


class O6ServoPressureTest(Node):
    def __init__(self, namespace):
        super().__init__("linkerhand_o6_servo_pressure_test")
        self.namespace = namespace.rstrip("/")
        self.actual = None
        self.status = None
        self.online = None
        self.ready = None
        self.pressure_status = None

        self.command_sequence = None
        self.command_sequence_advance = 0
        self.can_tx_count = None
        self.can_tx_advance = 0
        self.can_rx_count = None
        self.can_rx_advance = 0
        self.pressure_sequence = None
        self.pressure_sequence_advance = 0
        self.valid_pressure_sequences = 0
        self.last_valid_pressure_sequence = None
        self.maximum_pressure_age_ms = 0
        self.published_targets = 0
        self.current_target = None

        self.create_subscription(
            UInt8MultiArray,
            f"{self.namespace}/actual_position",
            self._on_actual,
            10,
        )
        self.create_subscription(
            UInt16MultiArray, f"{self.namespace}/status", self._on_status, 10
        )
        self.create_subscription(Bool, f"{self.namespace}/online", self._on_online, 10)
        self.create_subscription(Bool, f"{self.namespace}/ready", self._on_ready, 10)
        self.create_subscription(
            UInt16MultiArray,
            f"{self.namespace}/pressure_status",
            self._on_pressure_status,
            10,
        )
        self.target_publisher = self.create_publisher(
            UInt8MultiArray, f"{self.namespace}/target_position", 10
        )
        self.enable_client = self.create_client(SetBool, f"{self.namespace}/enable")

    def _on_actual(self, message):
        self.actual = list(message.data)

    def _on_status(self, message):
        values = list(message.data)
        self.status = values
        if len(values) <= CAN_RX_COUNT_INDEX:
            return

        command_sequence = values[COMMAND_SEQUENCE_INDEX]
        if self.command_sequence is not None:
            self.command_sequence_advance += counter_delta(
                self.command_sequence, command_sequence
            )
        self.command_sequence = command_sequence

        can_tx_count = values[CAN_TX_COUNT_INDEX]
        if self.can_tx_count is not None:
            self.can_tx_advance += counter_delta(self.can_tx_count, can_tx_count)
        self.can_tx_count = can_tx_count

        can_rx_count = values[CAN_RX_COUNT_INDEX]
        if self.can_rx_count is not None:
            self.can_rx_advance += counter_delta(self.can_rx_count, can_rx_count)
        self.can_rx_count = can_rx_count

    def _on_online(self, message):
        self.online = message.data

    def _on_ready(self, message):
        self.ready = message.data

    def _on_pressure_status(self, message):
        values = list(message.data)
        if len(values) < 4:
            return
        sequence, finger_mask, state, age_ms = values[:4]
        self.pressure_status = values[:4]
        self.maximum_pressure_age_ms = max(self.maximum_pressure_age_ms, age_ms)

        if self.pressure_sequence is not None:
            self.pressure_sequence_advance += counter_delta(
                self.pressure_sequence, sequence
            )
        self.pressure_sequence = sequence

        pressure_valid = (
            finger_mask == PRESSURE_FINGER_MASK
            and state == PRESSURE_COMPLETE_STATE
            and age_ms < PRESSURE_MAX_AGE_MS
        )
        if pressure_valid and sequence != self.last_valid_pressure_sequence:
            self.valid_pressure_sequences += 1
            self.last_valid_pressure_sequence = sequence

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
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.02, remaining))

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
            raise RuntimeError(f"使能操作失败：{message}")
        print(response.message)

    def send_target(self, target):
        message = UInt8MultiArray()
        message.data = target
        self.target_publisher.publish(message)
        self.current_target = list(target)
        self.published_targets += 1

    def report(self, started_at, baseline, can_bitrate, estimated_frame_bits):
        elapsed = max(0.001, time.monotonic() - started_at)
        published = self.published_targets - baseline[0]
        accepted = self.command_sequence_advance - baseline[1]
        can_tx = self.can_tx_advance - baseline[2]
        can_rx = self.can_rx_advance - baseline[3]
        pressure = self.pressure_sequence_advance - baseline[4]
        can_frame_rate = (can_tx + can_rx) / elapsed
        estimated_bus_load = (
            100.0 * can_frame_rate * estimated_frame_bits / can_bitrate
        )
        applied_sequence = None
        current_command_sequence = None
        applied_lag = None
        if self.status is not None and len(self.status) > COMMAND_SEQUENCE_INDEX:
            applied_sequence = self.status[APPLIED_SEQUENCE_INDEX]
            current_command_sequence = self.status[COMMAND_SEQUENCE_INDEX]
            applied_lag = counter_delta(applied_sequence, current_command_sequence)
        applied_matches = applied_lag == 0
        print(
            f"运行 {elapsed:.1f}s | 发布 {published} ({published / elapsed:.1f}Hz) | "
            f"接受 {accepted} | CAN TX/RX +{can_tx}/+{can_rx} | "
            f"CAN约{can_frame_rate:.1f}帧/s、负载约{estimated_bus_load:.2f}% | "
            f"压力序号 +{pressure}"
        )
        print(
            f"  AppliedCommandSequence={applied_sequence} | "
            f"CommandSequence={current_command_sequence} | "
            f"落后={applied_lag} | 匹配={applied_matches}"
        )
        print(
            f"  目标={self.current_target} 实际={self.actual} "
            f"压力状态={self.pressure_status}"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "六个O6位置通道同时连续往返，持续产生CAN位置命令并监测压力扫描。"
            "按Ctrl+C停止。"
        )
    )
    parser.add_argument(
        "--amplitude", type=int, default=100, help="相对启动位置的运动幅度，默认100"
    )
    parser.add_argument(
        "--rate", type=float, default=50.0, help="新位置目标发送频率Hz，默认50"
    )
    parser.add_argument(
        "--step", type=int, default=1, help="每条命令的位置增量，默认1"
    )
    parser.add_argument(
        "--report-interval", type=float, default=2.0, help="状态打印周期秒，默认2"
    )
    parser.add_argument(
        "--restore-wait", type=float, default=1.5, help="回原位后的等待时间秒，默认1.5"
    )
    parser.add_argument(
        "--can-bitrate", type=int, default=1_000_000, help="CAN波特率，默认1000000"
    )
    parser.add_argument(
        "--frame-bits",
        type=float,
        default=120.0,
        help="估算单帧线上比特数（含帧开销和位填充），默认120",
    )
    parser.add_argument(
        "--keep-enabled",
        action="store_true",
        help="如果脚本执行了使能，停止后仍保持使能",
    )
    parser.add_argument(
        "--namespace", default="/linkerhand_o6",
        help="O6 ROS namespace; use /linkerhand_o6_left for the left hand",
    )
    return parser.parse_args()


def validate_args(args):
    if not 1 <= args.amplitude <= 200:
        raise ValueError("--amplitude 必须在1～200之间")
    if not 1.0 <= args.rate <= 200.0:
        raise ValueError("--rate 必须在1～200 Hz之间")
    if not 1 <= args.step <= args.amplitude:
        raise ValueError("--step 必须在1和amplitude之间")
    if args.report_interval <= 0 or args.restore_wait < 0:
        raise ValueError("打印周期必须大于0，回原位等待不能小于0")
    if args.can_bitrate <= 0 or args.frame_bits <= 0:
        raise ValueError("CAN波特率和估算帧长度必须大于0")


def select_directions(origin, amplitude):
    directions = []
    for joint, value in enumerate(origin):
        if value >= amplitude:
            directions.append(-1)
        elif 255 - value >= amplitude:
            directions.append(1)
        else:
            raise RuntimeError(
                f"{JOINT_NAMES[joint]}当前位置{value}无法提供{amplitude}的运动范围"
            )
    return directions


def target_at_offset(origin, directions, offset):
    return [
        value + direction * offset
        for value, direction in zip(origin, directions)
    ]


def smooth_restore(node, origin, rate_hz, step):
    if node.current_target is None:
        return
    target = list(node.current_target)
    period_sec = 1.0 / rate_hz
    while rclpy.ok() and target != origin:
        for joint, reference in enumerate(origin):
            difference = reference - target[joint]
            if difference > 0:
                target[joint] += min(step, difference)
            elif difference < 0:
                target[joint] -= min(step, -difference)
        node.send_target(target)
        node.spin_for(period_sec)


def main():
    args = parse_args()
    try:
        validate_args(args)
    except ValueError as error:
        print(f"参数错误：{error}", file=sys.stderr)
        return 2

    # Keep the ROS context alive after Ctrl+C long enough to restore and disable safely.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = O6ServoPressureTest(args.namespace)
    enabled_by_script = False
    original = None
    running_trajectory = False
    baseline = None
    started_at = None
    exit_code = 0
    try:
        if not node.spin_until(
            lambda: (
                node.actual is not None
                and len(node.actual) == 6
                and node.status is not None
                and len(node.status) > CAN_RX_COUNT_INDEX
                and node.online is not None
                and node.ready is not None
                and node.pressure_status is not None
            ),
            5.0,
        ):
            raise RuntimeError("5秒内未收到完整的位置和压力状态")
        if not node.online:
            raise RuntimeError("O6 当前不在线")

        original = list(node.actual)
        directions = select_directions(original, args.amplitude)
        print(f"启动位置：{original}")
        print(f"运动方向：{directions}，幅度={args.amplitude}")
        print(f"初始压力状态：{node.pressure_status}")
        print(
            "主站持续保持压力RequestMask bit2；脚本持续发送位置伺服目标。"
            "按Ctrl+C停止并回原位。"
        )

        if not node.ready:
            node.set_enabled(True)
            enabled_by_script = True
            if not node.spin_until(lambda: node.ready is True, 3.0):
                raise RuntimeError("当前位置使能握手未进入 ready=true")
        else:
            print("O6 已经处于 ready=true，保持原有使能状态。")

        node.current_target = list(original)
        node.spin_for(0.5)
        baseline = (
            node.published_targets,
            node.command_sequence_advance,
            node.can_tx_advance,
            node.can_rx_advance,
            node.pressure_sequence_advance,
        )
        started_at = time.monotonic()
        next_send_at = started_at
        next_report_at = started_at + args.report_interval
        offset = 0
        direction = 1
        running_trajectory = True

        while rclpy.ok():
            offset += direction * args.step
            if offset >= args.amplitude:
                offset = args.amplitude
                direction = -1
            elif offset <= 0:
                offset = 0
                direction = 1

            node.send_target(target_at_offset(original, directions, offset))
            next_send_at += 1.0 / args.rate
            now = time.monotonic()
            if now > next_send_at + 1.0 / args.rate:
                next_send_at = now
            while rclpy.ok() and time.monotonic() < next_send_at:
                remaining = next_send_at - time.monotonic()
                rclpy.spin_once(node, timeout_sec=min(0.005, remaining))

            if time.monotonic() >= next_report_at:
                node.report(
                    started_at, baseline, args.can_bitrate, args.frame_bits
                )
                next_report_at += args.report_interval

    except KeyboardInterrupt:
        print("\n收到Ctrl+C，停止连续位置命令。")
    except (ExternalShutdownException, RuntimeError) as error:
        print(f"测试停止：{error}", file=sys.stderr)
        exit_code = 1
    finally:
        if baseline is not None and started_at is not None:
            node.report(started_at, baseline, args.can_bitrate, args.frame_bits)
        if running_trajectory and original is not None and rclpy.ok():
            try:
                print("正在平滑返回启动位置……")
                smooth_restore(node, original, args.rate, args.step)
                node.spin_for(args.restore_wait)
                print(f"已发送启动位置目标：{original}，当前反馈：{node.actual}")
            except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as error:
                print(f"回原位失败：{error}", file=sys.stderr)
                exit_code = 1

        print(
            "压力扫描汇总："
            f"PressureSequence累计前进={node.pressure_sequence_advance}，"
            f"完整有效序号={node.valid_pressure_sequences}，"
            f"最大PressureAgeMs={node.maximum_pressure_age_ms}"
        )
        print(
            "CAN计数汇总："
            f"TX累计增加={node.can_tx_advance}，RX累计增加={node.can_rx_advance}，"
            f"ROS目标发布={node.published_targets}"
        )
        if enabled_by_script and not args.keep_enabled and rclpy.ok():
            try:
                node.set_enabled(False)
                print("脚本已将 O6 失能。")
            except RuntimeError as error:
                print(f"退出时失能失败：{error}", file=sys.stderr)
                exit_code = 1
        elif enabled_by_script:
            print("O6 按 --keep-enabled 要求保持使能。")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
