#!/usr/bin/env python3
"""左臂 Pilz LIN：在本文件顶部修改末端目标和运动参数。"""

from planning_common import run


# ===== 运行参数：直接修改本文件，保存后重新运行即可 =====
VELOCITY_SCALE = 0.1             # 速度缩放 (0, 1]
ACCELERATION_SCALE = 0.01        # 加速度缩放；关节加速度基准仍为 2.0 rad/s²
PLANNING_TIME_S = 3.0            # 每次规划允许时间，秒
REPEAT = 1                       # 运行次数
PAUSE_S = 2.0                    # 执行前停顿，秒
EXECUTE = True                   # 默认直接执行
RETURN_TO_START = True           # 执行后规划返回本次运行前的左臂关节位置

# ===== 末端目标：直线终点 =====
TARGET_OFFSET_M = [0.0, 0.0, 0.02]  # 相对当前末端，base_link 下的 DX, DY, DZ，米
TARGET_POSITION_M = None          # 或填绝对 [x, y, z]，米；非 None 时优先于 OFFSET
TARGET_RPY_DEG = None             # 或填绝对 [roll, pitch, yaw]，度；None 保持当前姿态

RETURN_RRTCONNECT_RANGE = 0.0     # 返回起点使用 RRTConnect，0 自动选择扩展距离

PARAMETERS = {
    "velocity_scale": VELOCITY_SCALE,
    "acceleration_scale": ACCELERATION_SCALE,
    "planning_time": PLANNING_TIME_S,
    "repeat": REPEAT,
    "pause": PAUSE_S,
    "execute": EXECUTE,
    "return_to_start": RETURN_TO_START,
    "delta": TARGET_OFFSET_M,
    "target_position_m": TARGET_POSITION_M,
    "target_rpy_deg": TARGET_RPY_DEG,
    "return_rrtconnect_range": RETURN_RRTCONNECT_RANGE,
}


if __name__ == "__main__":
    raise SystemExit(run("lin", PARAMETERS, source_file=__file__))
