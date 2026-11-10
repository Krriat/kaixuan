#!/usr/bin/env python3
"""左臂 Cartesian Path：在本文件中修改途经点列表和插值参数。"""

from planning_common import run


# ===== 运行参数：直接修改本文件，保存后重新运行即可 =====
VELOCITY_SCALE = 0.1             # 速度缩放 (0, 1]
ACCELERATION_SCALE = 0.01        # 加速度缩放；关节加速度基准仍为 2.0 rad/s²
PLANNING_TIME_S = 3.0            # 每次规划允许时间，秒
REPEAT = 1                       # 运行次数
PAUSE_S = 2.0                    # 执行前停顿，秒
EXECUTE = True                   # 默认直接执行
RETURN_TO_START = True           # 执行后规划返回本次运行前的左臂关节位置

# ===== Cartesian 专用参数：按顺序经过的末端点 =====
# 每个点都相对于本次起始末端位置，base_link 下，单位米；不是相对前一个点。
WAYPOINT_OFFSETS_M = [
    [0.0, 0.0, 0.02],             # 当前示例：沿 +Z 移动 20 mm
    # [0.005, 0.0, 0.02],         # 可以继续添加途经点
]
WAYPOINT_POSITIONS_M = None       # 或填绝对坐标列表 [[x1,y1,z1], [x2,y2,z2], ...]
ORIENTATION_RPY_DEG = None        # 所有途经点的绝对 [roll,pitch,yaw]，度；None 保持起始姿态
MAX_STEP_M = 0.001                # 插值步长，米；0.001 = 1 mm
RELATIVE_JUMP_THRESHOLD = 5.0      # 相对关节跳变阈值，0 关闭相对检查
MAX_JOINT_JUMP_DEG = 10.0         # 相邻轨迹点最大单关节跳变，度
RETURN_RRTCONNECT_RANGE = 0.0     # 返回起点所用 RRTConnect 的扩展距离

PARAMETERS = {
    "velocity_scale": VELOCITY_SCALE,
    "acceleration_scale": ACCELERATION_SCALE,
    "planning_time": PLANNING_TIME_S,
    "repeat": REPEAT,
    "pause": PAUSE_S,
    "execute": EXECUTE,
    "return_to_start": RETURN_TO_START,
    "waypoint_offsets_m": WAYPOINT_OFFSETS_M,
    "waypoint_positions_m": WAYPOINT_POSITIONS_M,
    "target_rpy_deg": ORIENTATION_RPY_DEG,
    "max_step": MAX_STEP_M,
    "jump_threshold": RELATIVE_JUMP_THRESHOLD,
    "max_joint_jump_deg": MAX_JOINT_JUMP_DEG,
    "return_rrtconnect_range": RETURN_RRTCONNECT_RANGE,
}


if __name__ == "__main__":
    raise SystemExit(run("cartesian", PARAMETERS, source_file=__file__))
