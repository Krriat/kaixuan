# 七轴末端直线与外力扰动：小扰动基准验收

本文件保留原 7.48 cm、0.5 N、0.12 s 基准结果。
当前默认的 30 cm、5 N、1.5 s 演示及新视频见 [强扰动演示](VISIBLE_DEMO.md)。

2026-11-06，Conda `mujoco_test`；这是离线仿真结果。
旧的第七轴回零实验及其生成结果已删除，本文件只记录新的七轴实验。

## 环境与复现

依赖：python 3.10.20, mujoco 3.3.7, numpy 2.2.6, scipy 1.15.3, pinocchio 4.0.0。
最终结果目录：`artifacts/cartesian_lqr_20261106_final/`（生成文件，Git 忽略）。

```bash
conda activate mujoco_test
python -m unittest discover -s tests -v
python -m unittest discover -s experiments/01_qp/tests -v
python -m unittest discover -s experiments/02_lqr/tests -v
# 新建一个尚不存在的输出目录：
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl \
  python -m kaixuan_control_lab.run_lqr --config config/cartesian_lqr.yaml --pinocchio --video \
  --output artifacts/cartesian_line_reproduction
```

6 项基础、8 项 QP、11 项新 LQR 测试全部通过，共 25 项。
独立的整段二次问题直接求解与 Riccati 反馈序列一致；外力映射另由 `mj_applyFT` 核对。
32 个样本的 MuJoCo/Pinocchio 模型交叉检查通过。
完整网格 MP4 已生成并检查；数值实验、MP4 和约 5 秒桌面回放的组合命令退出码为 0。
初次回放曾遇到 GLX 关闭竞争，已修复为等待本次窗口创建的线程完成退出；最终重跑正常关闭。

## 参考与设计检查

- `line_length_m`：`0.07483314773547883`。
- `max_position_error_m`：`8.141701868436414e-11`。
- `max_orientation_error_rad`：`1.2989978323297694e-10`。
- `joint_excursion_deg`：`[6.439254518517529, 5.34395743454473, 0.620771913076791, 7.162012643009405, 8.449146192992933, 10.75099041263267, 0.8664034831227297]`。
- `horizon_steps`：`500`。
- `terminal_dare_relative_residual`：`1.98172371804948e-15`。
- `max_control_stationarity_relative_residual`：`2.1267673500648603e-17`。
- `terminal_spectral_radius`：`0.960577526458227`。
- `linearization_directional_relative_error`：`2.91663689685332e-10`。
- `offline_design_wall_s`：`0.3377572240005975`。

以上末端误差为参考样条本身在 1 ms 采样点的几何误差；不能理解为真实机械臂精度。
同一份 URDF 的跨引擎最大绝对误差：

- `mass`：`5.92639e-10`。
- `gravity`：`6.21725e-15`。
- `bias`：`8.73861e-11`。
- `position`：`4.71845e-16`。
- `rotation`：`1.77636e-15`。
- `jacobian`：`1.44329e-15`。
- `inverse_dynamics`：`4.5395e-10`。

## 主推力实验：峰值 0.5 N，持续 0.12 s

初值、参考、外力波形与限制相同；控制器不知道外力。
较快 PD 同时列出，避免只与较慢 PD 比较。

| 方法 | 峰值位置误差 mm | 全程 RMS mm | 脉冲后恢复到 1 mm 内 s | 共同完整二次代价 |
| --- | ---: | ---: | ---: | ---: |
| 参考前馈＋独立 PD，ω=12 | 5.180789 | 0.949666 | 0.284 | 0.38591585 |
| 计算力矩＋PD，ω=12 | 4.387222 | 0.766390 | 0.260 | 0.33033671 |
| 时变 LQR | 0.885515 | 0.094252 | 0.000 | 0.01091515 |
| 计算力矩＋PD，ω=80 | 0.311039 | 0.031128 | 0.000 | 0.00212392 |

恢复时间为 0 表示误差始终在 1 mm 内，不表示瞬间纠偏。
主推力场景没有电机力矩饱和。
在理想模型下，较快的计算力矩 PD **优于当前 LQR** 的这次扰动结果。
未知外力并未纳入 LQR 设计；这项实际非线性运行的二次代价也不是 LQR 理论最优值。

## 延迟与压力场景

延迟组只改变物理执行器：2 ms 延迟加 5 ms 一阶时间常数；控制设计仍使用理想模型。

| 场景 | 方法 | 状态 | 记录到的峰值误差 mm | 结束/停机时间 s |
| --- | --- | --- | ---: | ---: |
| delayed_nominal | 参考前馈＋独立 PD，ω=12 | completed | 0.149791 | 5.000 |
| delayed_nominal | 计算力矩＋PD，ω=12 | completed | 0.146560 | 5.000 |
| delayed_nominal | 时变 LQR | completed | 0.007872 | 5.000 |
| delayed_nominal | 计算力矩＋PD，ω=80 | guard_stopped | 4.844246 | 1.729 |
| delayed_push | 参考前馈＋独立 PD，ω=12 | completed | 5.400257 | 5.000 |
| delayed_push | 计算力矩＋PD，ω=12 | completed | 4.720669 | 5.000 |
| delayed_push | 时变 LQR | completed | 1.010783 | 5.000 |
| delayed_push | 计算力矩＋PD，ω=80 | guard_stopped | 4.844246 | 1.729 |
| stress_push | 参考前馈＋独立 PD，ω=12 | guard_stopped | 2.521649 | 1.852 |
| stress_push | 计算力矩＋PD，ω=12 | guard_stopped | 2.558482 | 1.852 |
| stress_push | 时变 LQR | completed | 3.601116 | 5.000 |
| stress_push | 计算力矩＋PD，ω=80 | completed | 1.248454 | 5.000 |

ω=80 的计算力矩 PD 在延迟条件下于 1.729 s 触发速度保护，早于 1.8 s 的推力开始。
有/无推力的两个延迟运行在此前相同，所以不能把它的停机归因于推力。
较慢 PD 和当前 LQR 都完成了该延迟组；这只说明这些设置的表现，不能概括为 LQR 普遍更鲁棒。
2 N 压力测试中，默认两种 PD 触发速度保护，LQR 和 ω=80 的计算力矩 PD 完成。
停机数据保留最后越界状态、请求/实际力矩、保护原因，并标记代价被截断。

## PD 扫描与所有检查

扫描共同自然频率 8、12、20、30、40、60、80 rad/s，阻尼比固定为 1。
名义/推力场景都保存到 `pd_sweep.json/csv`；不只保留有利于 LQR 的设置。
独立关节 PD 在较高频率下会先触发保护；扫描不等于每轴增益都经过独立充分优化。

实际独立运行共 46 次：completed=32, guard_stopped=14。
重复使用的默认 PD 扫描项和快速 PD 的名义/推力项只计算一次，避免重复计数。
15 个常规主运行均完成；所有主运行实际电机力矩都在限值内。
常规主运行最大最终位置误差 `5.6755368e-06 mm`。

- `model_validation`：`True`。
- `reference_geometry`：`True`。
- `multiple_joints_move`：`True`。
- `all_regular_cases_complete`：`True`。
- `all_regular_final_positions`：`True`。
- `nominal_tracking`：`True`。
- `push_tracking`：`True`。
- `motor_torque_limits`：`True`。
- `regular_state_limits`：`True`。
- `stress_outcomes_recorded`：`True`。
- `pd_sweep_outcomes_recorded`：`True`。
- `strong_pd_outcomes_recorded`：`True`。
- `linearization`：`True`。
- `riccati`：`True`。

程序总体 PASS 的含义是：规定的常规实验通过，压力/高增益实验的完成或保护停机均被正确记录。
它不要求所有压力测试成功，也不要求 LQR 优于每一种对照。
该轮生成时源码哈希核对通过；后续视频和运行入口已更新，旧哈希不代表当前源码版本。三种控制器的主推力样本逐点一致，NPZ 重算指标与摘要一致。
MP4 为 960×720、151 帧、约 5.03 秒；轨迹、关节/力矩、增益和扫描图已目视检查。

## 解释边界

MuJoCo 与 Pinocchio 一致只证明模型转换和坐标约定一致，不证明 CAD 参数真实。
惯量估计、未辨识转子惯量、理想摩擦、无碰撞/接触等假设见 `MODEL_ASSUMPTIONS.md`。
未测试任何真实电机内环或厂商 CSP；没有连接机器人，也没有迁移控制器。
