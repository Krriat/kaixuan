# 可选的 1 kHz 原生 LQI 实验入口

这是本轮保留下来的可复现实验基线，**运动中的粘滑尚未解决**。默认的 Python LQI
入口没有替换。试验和对照结果见 [振动排查记录](CST_LQR_VIBRATION.md)。
末端误差来自编码器和 URDF FK；没有外部末端测量或现场目视验收。

## 控制实际运行在哪里

Python 在使能前完成 URDF/Pinocchio 建模、碰撞检查、轨迹规划和 Riccati 递推，
随后以 500 Hz 发送参考位置、速度、动力学/摩擦前馈、反馈增益和积分逆增益。
`kaixuan_can_hardware/LqrTorqueController` 在 ros2_control 的 1 kHz 更新线程里
读取最新硬件位置、速度，计算实际力矩。Python 不再闭合力矩反馈环。

实验基线使用 25 维状态：7 个位置误差、7 个速度误差、7 个积分误差，以及 J4
的一阶执行器状态和 3 个延迟队列状态。执行器状态由上一次限幅后的指令推进，
与前馈工作点的差参与反馈。输入响应模型是“命令到驱动器报告力矩”的近似，
包含通信和报告滤波；不是独立电流环或机械惯量辨识。

静止/运动阶段切换时保留积分的物理力矩贡献。幅值、增长/退力矩变化率、积分抗饱和、
45°/s 速度检查、8° 跟踪误差检查和命令超时均在运行中执行。原生控制周期超过
10 ms 会锁定撤销输出。硬件层仍保留原来的限幅和保护。限幅不属于无约束 LQR 的优化约束。

原生控制器要先收到本次激活后的明确零力矩握手，才接受反馈输出。失效时写入 NaN
撤销 effort，硬件按配置去使能。该实验入口结束方式为 CW0；本机去使能后可能被动下滑，
CW0 不能作为机械抱闸已经闭合的证明。

## 编译与启动

在机器人工作区执行：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
colcon build --base-paths src/kaixuan_can_hardware src/kaixuan3_config \
  --packages-select kaixuan_can_hardware kaixuan3_config --symlink-install
source install/setup.bash
ros2 launch kaixuan3_config cst_cartesian_lqr_native.launch.py
```

启动只建立左臂 CST 和控制器通道，不发送有限力矩、不使能电机。
第 4 轴厂家电流环仍使用 P=2000、I=300；launch 不改写厂家 PI 或 EEPROM。

另一终端读取一个**新的**未使能静止姿态，再运行有返回过程的实验：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control switch_controllers --deactivate torque_controller
ros2 run kaixuan3_config cst_standby_snapshot.py --output /tmp/left_standby_new.json
ros2 control switch_controllers --activate torque_controller
OPENBLAS_NUM_THREADS=1 ros2 run kaixuan3_config cst_cartesian_z.py \
  --config src/kaixuan3_config/config/cst_cartesian_lqr_native.yaml \
  --resume-from /tmp/left_standby_new.json --execute --finish return
```

快照工具只读状态，要求连续 3 秒静止、控制器未占有 effort、七轴未使能、零目标力矩且
无故障。已有输出文件会被拒绝覆盖；每次改用新文件名。规划完成后还会再次核对当前姿态。
不要复用历史日志中的姿态文件替代实时读取。

执行过程为零力矩握手、5 秒渐入、30 秒进入
`[-20, 20, 30, 50, -10, -20, 0]°`、保持朝向沿 world Z 上移 10 cm、逆路径、
30 秒回到本次启动姿态、5 秒卸载和去使能。直线运动为 10 秒，另含首尾各 1 秒保持。
可以先添加 `--stop-after hold` 验证本次初始姿态的 10 秒保持。
右臂未校准，其几何体未纳入碰撞检查；本轮试验依据用户确认的左臂区域空旷条件。

`shadow_only:=true` 会执行全部原生反馈计算，但物理 effort 始终为 NaN，零握手也不会使能。
正常 `--execute` 入口会拒绝这种模式，避免把影子计算误认为实机运动。

## 参数与证据

- 软件候选：`config/cst_cartesian_lqr_native.yaml`。
- 硬件实验限幅：`kaixuan_can_hardware/config/cartesian_native_torque_limits.yaml`。
  J1/J3/J4 总力矩上限 20 N·m，J2 为 13.6 N·m，J5–J7 为 3 N·m；
  J4 增长仍为 160 N·m/s，退力矩为 640 N·m/s。
- 每次输出目录保存活动 URDF、规划和配置摘要、原生库哈希、增益、原生计算的逐周期状态、
  实际指令、报告力矩及退出状态。采样日志不是每一个 1 kHz 周期的完整记录。
- 修改输入延迟、时间常数、积分/速度/力矩限制等原生静态参数后，需要重新配置或重启控制器。
  `planning_config_sha256` 只用于核对规划配置，不能用改摘要的方式冒充静态参数已经生效。
- 诊断线程抢锁失败时做少量非实时重试，并报告 `diagnostic_snapshot_retries` 和
  `diagnostic_snapshot_drops`。保护仍按真实样本年龄检查；重试不保证任意系统负载下的发布期限。

不能因为一轮路径误差通过，就判定手臂已经平顺或在所有姿态下稳定。
