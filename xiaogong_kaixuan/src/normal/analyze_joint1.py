#!/usr/bin/env python3
"""
右臂1号关节跟踪性能分析工具（双模式）

模式1 - 实时采集（推荐）:
    python3 analyze_joint1.py record [输出CSV路径]
    # 在另一个终端运行运动程序，Ctrl+C 停止采集

模式2 - 分析已有CSV:
    python3 analyze_joint1.py plot <CSV文件路径>
"""

import sys
import os
import shlex


def _force_udp_only_local_transport() -> None:
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")


_force_udp_only_local_transport()


def _ensure_ros_runtime_env() -> None:
    if os.environ.get("_ANALYZE_ROS_ENV_READY") == "1":
        return
    if os.environ.get("AMENT_PREFIX_PATH") and os.environ.get("LD_LIBRARY_PATH"):
        return

    script_path = os.path.abspath(__file__)
    quoted_args = " ".join(shlex.quote(arg) for arg in sys.argv[1:])
    command = (
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
        "source /home/niic/cxl/ros2_control/install/setup.bash >/dev/null 2>&1 && "
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && "
        f"export _ANALYZE_ROS_ENV_READY=1 && python3 {shlex.quote(script_path)}"
    )
    if quoted_args:
        command += f" {quoted_args}"
    os.execv("/bin/bash", ["bash", "-lc", command])


_ensure_ros_runtime_env()

# joint_config.py 与本脚本同目录，Python 自动将脚本目录加入 sys.path
from joint_config import JOINT_IDX, JOINT_NAME, CONTROLLER_TOPIC  # noqa: E402


# ─────────────────────────────────────────────
# 统计 + 绘图（两种模式共用）
# ─────────────────────────────────────────────

def print_stats(df):
    import numpy as np
    err = df["error_pos"].abs()
    print(f"\n{'='*50}")
    print(f"  {JOINT_NAME} 跟踪误差统计  (共 {len(df)} 帧)")
    print(f"{'='*50}")
    print(f"  最大误差  : {float(np.degrees(err.max())):.3f} deg  ({float(err.max()):.4f} rad)")
    print(f"  均方根误差: {float(np.degrees(np.sqrt((err**2).mean()))):.3f} deg  (RMSE)")
    print(f"  平均误差  : {float(np.degrees(err.mean())):.3f} deg")
    print(f"{'='*50}\n")


def plot(df, save_path=None):
    import numpy as np
    import matplotlib
    if save_path:
        matplotlib.use("Agg")
    else:
        matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    # 设置中文字体
    plt.rcParams["font.family"] = ["Noto Sans CJK JP", "WenQuanYi Micro Hei",
                                   "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    t = np.array(df["time_s"], dtype=float)
    desired_deg = np.degrees(np.array(df["desired_pos"], dtype=float))
    actual_deg  = np.degrees(np.array(df["actual_pos"],  dtype=float))
    err_deg     = np.degrees(np.array(df["error_pos"],   dtype=float))

    # 速度：对位置求导（deg/s），用中心差分，端点用前/后向差分
    desired_vel = np.gradient(desired_deg, t)
    actual_vel  = np.gradient(actual_deg,  t)

    from matplotlib.ticker import MultipleLocator

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f"{JOINT_NAME} 跟踪分析", fontsize=14)

    ax = axes[0]
    ax.plot(t, desired_deg, label="指令 (deg)", linewidth=1.5, color="royalblue")
    ax.plot(t, actual_deg,  label="反馈 (deg)", linewidth=1.5, color="tomato", linestyle="--")
    ax.set_ylabel("关节角度 (deg)"); ax.legend(); ax.grid(True, alpha=0.4)
    ax.set_title("位置：指令 vs 反馈")

    ax = axes[1]
    ax.plot(t, desired_vel, label="指令速度 (deg/s)", linewidth=1.5, color="royalblue")
    ax.plot(t, actual_vel,  label="反馈速度 (deg/s)", linewidth=1.5, color="tomato", linestyle="--")
    ax.set_ylabel("角速度 (deg/s)"); ax.legend(); ax.grid(True, alpha=0.4)
    ax.set_title("速度：指令 vs 反馈（位置求导）")

    ax = axes[2]
    ax.plot(t, err_deg, color="darkorange", linewidth=1.2, label="误差 (deg)")
    ax.set_ylabel("位置误差 (deg)"); ax.set_xlabel("时间 (s)")
    ax.legend(); ax.grid(True, alpha=0.4)
    ax.set_title("跟踪误差")

    axes[2].xaxis.set_major_locator(MultipleLocator(1))
    axes[2].xaxis.set_major_formatter(lambda x, _: f"{int(x)}")

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"图表已保存: {save_path}")
    else:
        plt.show()


# ─────────────────────────────────────────────
# 模式1：实时采集（rclpy 直接订阅）
# ─────────────────────────────────────────────

def cmd_record(csv_path: str):
    """订阅 /right_arm_controller/controller_state，实时写 CSV"""
    import csv
    import time
    try:
        import rclpy
        from rclpy.node import Node
        from control_msgs.msg import JointTrajectoryControllerState
    except ImportError as e:
        print(f"缺少依赖: {e}")
        print("请先 source install/setup.bash")
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node("joint1_recorder")
    t0: list[float | None] = [None]
    rows_written = [0]

    f = open(csv_path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["time_s", "desired_pos", "actual_pos", "error_pos"])

    def cb(msg: JointTrajectoryControllerState):
        now = time.monotonic()
        if t0[0] is None:
            t0[0] = now
        t = now - t0[0]

        def safe(arr, idx):
            return float(arr[idx]) if arr and len(arr) > idx else float("nan")

        desired = getattr(msg, "reference", None) or getattr(msg, "desired", None)
        actual  = getattr(msg, "feedback",  None) or getattr(msg, "actual",  None)
        error   = getattr(msg, "error", None)

        if desired is None or actual is None:
            return

        dp  = safe(desired.positions,  JOINT_IDX)
        ap  = safe(actual.positions,   JOINT_IDX)
        ep  = safe(error.positions,    JOINT_IDX) if error else (ap - dp)
        writer.writerow([f"{t:.4f}", dp, ap, ep])
        rows_written[0] += 1
        if rows_written[0] % 100 == 0:
            f.flush()
            print(f"\r  采集中... {rows_written[0]} 帧  t={t:.1f}s", end="", flush=True)

    node.create_subscription(JointTrajectoryControllerState, CONTROLLER_TOPIC, cb, 10)
    print(f"订阅 {CONTROLLER_TOPIC}")
    print(f"数据写入: {csv_path}")
    print("在另一个终端运行运动程序，按 Ctrl+C 停止采集\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        f.flush(); f.close()
        node.destroy_node(); rclpy.shutdown()
        print(f"\n采集完成，共 {rows_written[0]} 帧 → {csv_path}")
        if rows_written[0] > 0:
            _plot_from_csv(csv_path)


def _plot_from_csv(csv_path: str):
    import pandas as pd
    df = pd.read_csv(csv_path)
    print_stats(df)
    img = csv_path.replace(".csv", ".png")
    plot(df, save_path=img)


# ─────────────────────────────────────────────
# 模式2：分析已有 CSV
# ─────────────────────────────────────────────

def cmd_plot(csv_path: str):
    import pandas as pd
    df = pd.read_csv(csv_path)
    print(f"读取 {len(df)} 帧数据")
    print_stats(df)
    img = csv_path.replace(".csv", ".png")
    plot(df, save_path=img)


# ─────────────────────────────────────────────
# 主入口
# ─────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)

    mode = sys.argv[1]

    if mode == "record":
        out = sys.argv[2] if len(sys.argv) > 2 else \
              os.path.expanduser(f"~/joint_logs/joint1_live_{__import__('time').strftime('%Y%m%d_%H%M%S')}.csv")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        cmd_record(out)

    elif mode == "plot":
        if len(sys.argv) < 3:
            print("用法: python3 analyze_joint1.py plot <CSV路径>"); sys.exit(1)
        cmd_plot(sys.argv[2])

    else:
        print(__doc__)


if __name__ == "__main__":
    main()
