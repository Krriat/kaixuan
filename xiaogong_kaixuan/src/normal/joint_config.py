#!/usr/bin/env python3
"""
════════════════════════════════════════════════════
  统一配置文件 —— 修改这里即可同步所有脚本
════════════════════════════════════════════════════
"""

# ══ 关节选择 ══════════════════════════════════════
ARM       = "right"   # "right" 或 "left"
JOINT_IDX = 2
# 0~6  (0=joint1, 1=joint2, ..., 6=joint7)

# ══ 运动参数 ══════════════════════════════════════
MAX_SPEED_DEG_PER_SEC = 30.0   # 100% 速度对应的角速度 (°/s)
FROM_DEG   = 0.0               # 起始角度（度）
TO_DEG     = -90.0              # 目标角度（度）
SPEED_TO   = 0.50              # 去程速度百分比 (0.0 ~ 1.0)
SPEED_BACK = 0.20              # 回程速度百分比 (0.0 ~ 1.0)
SLEEP_S    = 3.0               # 到达目标后等待时间（秒）

# ══ 自动派生（无需修改）══════════════════════════
_ARM_JOINTS = {
    "right": ["right_joint1", "right_joint2", "right_joint3", "right_joint4",
               "right_joint5", "right_joint6", "right_joint7"],
    "left":  ["left_joint1",  "left_joint2",  "left_joint3",  "left_joint4",
               "left_joint5",  "left_joint6",  "left_joint7"],
}

JOINT_NAME       = _ARM_JOINTS[ARM][JOINT_IDX]
CONTROLLER_NAME  = f"{ARM}_arm_controller"
CONTROLLER_TOPIC = f"/{CONTROLLER_NAME}/controller_state"
