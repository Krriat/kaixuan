#!/usr/bin/env python3
"""Run after sourcing scripts/environment.sh. Add --execute to move the real arm."""

import argparse
from pathlib import Path
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from motion_client import ArmClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    arm = ArmClient("left")
    try:
        # Your code goes here. Values use the same ROS coordinates as RViz.
        target_deg = arm.current_joint_angles(degrees=True)
        print("当前角度 (°)：", target_deg)
        target_deg[3] += 1.0  # Example: left_joint4 +1 degree, hold the other targets.
        arm.move_joints(target_deg, degrees=True, execute=args.execute,
                        velocity=0.1, acceleration=0.1)
        print("执行成功" if args.execute else "规划成功；加 --execute 才会执行")
    except KeyboardInterrupt:
        arm.cancel()
        return 130
    finally:
        arm.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
