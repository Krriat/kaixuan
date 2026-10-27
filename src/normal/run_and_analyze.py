#!/usr/bin/env python3
"""
整合脚本：自动录制关节数据 → 执行关节动作序列 → 绘制分析图

运行方式：
    python3 run_and_analyze.py

所有配置在 joint_config.py 中修改。
"""

import os
import shlex
import signal
import subprocess
import sys
import time


def _force_udp_only_local_transport() -> None:
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")


_force_udp_only_local_transport()


def _ensure_ros_runtime_env() -> None:
    if os.environ.get("_RUN_AND_ANALYZE_ROS_ENV_READY") == "1":
        return
    if os.environ.get("AMENT_PREFIX_PATH") and os.environ.get("LD_LIBRARY_PATH"):
        return

    script_path = os.path.abspath(__file__)
    command = (
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
        "source /home/niic/cxl/ros2_control/install/setup.bash >/dev/null 2>&1 && "
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && "
        f"export _RUN_AND_ANALYZE_ROS_ENV_READY=1 && python3 {shlex.quote(script_path)}"
    )
    os.execv("/bin/bash", ["bash", "-lc", command])


_ensure_ros_runtime_env()

# joint_config.py 与本脚本同目录，直接导入
from joint_config import JOINT_NAME, CONTROLLER_NAME  # noqa: E402

# ══ 编排参数（如需修改节奏，改这里；关节 / 运动参数改 joint_config.py）══
RECORD_WARM_UP_S = 2.0   # 录制进程启动后，等待其就绪再开始运动（秒）
STOP_WAIT_S      = 5.0   # 发送 SIGINT 后等待录制进程退出的超时（秒）
# ══════════════════════════════════════════════════════════════════════════════

_DIR = os.path.dirname(os.path.abspath(__file__))


def _script(name: str) -> str:
    return os.path.join(_DIR, name)


def main() -> None:
    # ── 生成本次录制的 CSV 路径 ─────────────────────────────────────────────
    ts = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.expanduser(f"~/joint_logs/{JOINT_NAME}_{ts}.csv")
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    print("=" * 60)
    print(f"  关节   : {JOINT_NAME}  ({CONTROLLER_NAME})")
    print(f"  CSV    : {csv_path}")
    print("=" * 60)

    # ── 第 1 步：启动录制进程 ───────────────────────────────────────────────
    print(f"\n[1/4] 启动数据录制...")
    recorder = subprocess.Popen(
        [sys.executable, _script("analyze_joint1.py"), "record", csv_path],
        env=os.environ.copy(),
    )

    print(f"      等待 {RECORD_WARM_UP_S:.1f}s 让录制进程就绪...")
    time.sleep(RECORD_WARM_UP_S)

    # ── 第 2 步：执行关节运动 ───────────────────────────────────────────────
    print(f"\n[2/4] 执行关节动作序列...")
    motion = subprocess.run(
        [sys.executable, _script("run_joint_sequence.py")],
        env=os.environ.copy(),
    )
    if motion.returncode != 0:
        print(f"      [警告] run_joint_sequence.py 退出码 = {motion.returncode}")

    # ── 第 3 步：停止录制 ───────────────────────────────────────────────────
    print(f"\n[3/4] 停止录制...")
    if recorder.poll() is None:                  # 进程仍在运行
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=STOP_WAIT_S)
        except subprocess.TimeoutExpired:
            print("      [警告] 录制进程未响应 SIGINT，强制终止")
            recorder.kill()
            recorder.wait()
    print(f"      录制完成 → {csv_path}")

    # ── 第 4 步：绘制分析图 ─────────────────────────────────────────────────
    print(f"\n[4/4] 生成分析图表...")
    subprocess.run(
        [sys.executable, _script("analyze_joint1.py"), "plot", csv_path],
        env=os.environ.copy(),
    )

    img_path = csv_path.replace(".csv", ".png")
    print(f"\n完成！分析图表已保存至: {img_path}")


if __name__ == "__main__":
    main()
