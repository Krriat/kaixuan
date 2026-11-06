# 看得见的直线运动与侧向推力演示

这是根据“原视频运动距离短、推力太短且看不清”的反馈重做的演示。
默认配置已经切换到 `config/cartesian_lqr_demo.yaml`；原小扰动基准保留在 `config/cartesian_lqr.yaml`。

## 直接看新版视频

[新版讲解视频](../../artifacts/cartesian_lqr_visible_demo/push_tvlqr.mp4)；
[施力时刻截图](../../artifacts/cartesian_lqr_visible_demo/replay_preview.png)；
[撤力后截图](../../artifacts/cartesian_lqr_visible_demo/replay_recovery.png)。

视频是 1600×900、约 16 秒、0.5 倍速：

- 第 0–2 秒：准备，观察绿色目标路径。
- 第 2–6 秒：左臂沿直线前进。
- **第 6–9 秒：运动途中侧推，出现红色提示和箭头。** 对应仿真 3.00–4.50 秒。
- 第 9 秒以后：外力撤去，左臂继续前进并恢复跟踪；右侧误差曲线下降。

绿色线/绿点表示目标路径/当前目标，蓝色表示实际左臂与末端轨迹，红箭头表示实际施加的外力。
固定机身和右臂仅在画面中变为半透明，便于看清左臂；没有更改其物理参数。
右侧路径投影在两个方向使用相同长度比例，机器人位姿与偏差均没有人为放大。
右侧两条同步曲线分别显示施加的力和实际末端误差，灰色误差线是同控制器的无外力对照。

## 参数变化

| 项目 | 原视频 | 新演示 |
| --- | ---: | ---: |
| 直线路程 | 7.48 cm | 29.93 cm |
| 运动时长 | 3 s | 5 s |
| 推力峰值 | 0.5 N | 5 N |
| 推力持续 | 0.12 s | 1.5 s |
| 仿真施力时段 | 1.80–1.92 s | 3.00–4.50 s |
| 回放速度 | 1× | 0.5× |

推力增大 10 倍、持续时间增大 12.5 倍，sin² 脉冲的冲量由 0.03 增至 3.75 N·s。
施力期间目标本身仍沿直线前进约 15.55 cm，控制器不知道外力；没有暂停参考来等待扰动结束。

## 复现

在 `control_lab/` 中：

```bash
conda activate mujoco_test
# 默认就是新演示；输出目录必须尚不存在。
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python -m kaixuan_control_lab.run_lqr --pinocchio --video \
  --output artifacts/cartesian_lqr_demo_reproduction

# 仅打开现有记录的交互回放，不重新运行控制器：
python -m kaixuan_control_lab.lqr_replay artifacts/cartesian_lqr_visible_demo

# 原小扰动基准仍可显式运行：
python -m kaixuan_control_lab.run_lqr --config config/cartesian_lqr.yaml --pinocchio
```

## 本机结果

2026-11-06，Conda `mujoco_test`；MuJoCo 3.3.7、Pinocchio 4.0.0、Python 3.10.20。
6 项基础、8 项 QP、12 项 LQR/视频投影测试通过，共 26 项；32 个样本的跨引擎检查通过。
额外检查了两种脉冲的支持时段、冲量、路径投影长度和输入向量不被修改。

| LQR 场景 | 状态 | 最大偏离 mm | 最大关节速度 rad/s | 力矩饱和步数 |
| --- | --- | ---: | ---: | ---: |
| nominal | completed | 0.008 | 0.237 | 0 |
| push | completed | 30.908 | 1.188 | 0 |
| reverse_push | completed | 13.271 | 0.820 | 0 |
| delayed_nominal | completed | 0.018 | 0.237 | 0 |
| delayed_push | completed | 31.217 | 1.187 | 0 |
| stress_push | guard_stopped | 12.837 | 1.583 | 0 |

主视频完整运行 8 秒，实际最大偏离 **30.908 mm**，未触发速度保护，未发生力矩饱和。
最终位置误差 8.143327e-09 mm。恢复时间指标为撤力后 0.016 s 进入并保持在 1 mm 内。
参考本身的直线/姿态误差、Riccati 残差、全部指标、保护状态、源码/模型哈希保存在新目录的 `summary.json`。

强扰动下普通 PD 对照会提前触发速度保护，其较小的记录峰值属于不完整轨迹，不能与完成的 LQR 直接排名。
80 rad/s 的计算力矩 PD 仍在理想执行器下有更小的位置偏差；所有这些结果照实保留。
演示配置的 PASS 只要求声明的 LQR 常规场景完成并满足阈值，对照与压力测试允许记录保护停机；
这不表示每种控制器均成功。原小扰动基准仍要求三种控制器的常规运行完成。

保留原有 90°/s 速度保护、位置界和力矩限值；没有为了展示而修改物理保护。
这是离线刚体仿真，原来的惯量/摩擦/无接触假设继续适用，没有连接真实机器人。
