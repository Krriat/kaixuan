#!/usr/bin/env python3
"""左臂 RRT*：目标、运行参数和算法参数均在本文件顶部修改。"""

from planning_common import run


# ===== 运行参数：直接修改本文件，保存后重新运行即可 =====
VELOCITY_SCALE = 0.3             # 速度缩放 (0, 1]
ACCELERATION_SCALE = 0.01        # 加速度缩放；关节加速度基准仍为 2.0 rad/s²
PLANNING_TIME_S = 3.0            # 每次规划允许时间，秒
REPEAT = 1                       # 运行次数
PAUSE_S = 2.0                    # 执行前停顿，秒
EXECUTE = True                   # 默认直接执行
RETURN_TO_START = False           # 执行后规划返回本次运行前的左臂关节位置

# ===== 唯一目标输入：七个绝对关节角，单位度 =====
# 按 left_joint1 到 left_joint7 的顺序填写；运行前必须填满七个值。
# 填写格式：[关节1角度, 关节2角度, 关节3角度, 关节4角度, 关节5角度, 关节6角度, 关节7角度]
TARGET_JOINTS_DEG = [-20, 20, 30, 50, -10, -20, 0]


# ===== RRT* 专用参数 =====
PLANNER_PARAMETERS = {
    "range": 0.0,                  # 配置空间扩展距离；0 由 OMPL 自动选择，不是末端位移
    "goal_bias": 0.05,             # 朝目标采样的概率 [0, 1]
    "delay_collision_checking": 1, # 延迟碰撞检查：0 或 1
}

RETURN_RRTCONNECT_RANGE = 0.0     # 仅用于返回起点的 RRTConnect

PARAMETERS = {
    "velocity_scale": VELOCITY_SCALE,
    "acceleration_scale": ACCELERATION_SCALE,
    "planning_time": PLANNING_TIME_S,
    "repeat": REPEAT,
    "pause": PAUSE_S,
    "execute": EXECUTE,
    "return_to_start": RETURN_TO_START,
    "joint_goal_deg": TARGET_JOINTS_DEG,
    "planner_parameters": PLANNER_PARAMETERS,
    "return_rrtconnect_range": RETURN_RRTCONNECT_RANGE,
}


if __name__ == "__main__":
    raise SystemExit(run("rrtstar", PARAMETERS, source_file=__file__))
