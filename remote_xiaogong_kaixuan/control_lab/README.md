# 凯旋力矩控制实验室

这是电脑端的 **离线 MuJoCo 实践环境**，目标是逐步掌握 QP、LQR、MPC、WBC。
模型取自 `../model/robot.urdf`，不依赖机器人上的安装路径、ROS 或 EtherCAT。
全部命令从本目录执行。模型、参数、算法和实验日志统一采用关节输出轴的 SI 单位。

已经实现阶段 00：URDF → MJCF、七轴力矩执行器、仿真状态接口、重力/零力矩基线、
可选执行器延迟与一阶响应、日志、独立动力学交叉验证。
LQR/MPC/WBC 留作后续分阶段实现，具体见 [ROADMAP.md](ROADMAP.md)。
给本地 Codex 的上下文和第一条任务见 [LOCAL_CODEX_HANDOFF.md](LOCAL_CODEX_HANDOFF.md)。

## 电脑端启动

在已获取本次修改的电脑端仓库中：

```bash
cd remote_xiaogong_kaixuan/control_lab
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt

# 无窗口的数值检查和 5 秒重力补偿基线
python -m unittest discover -s tests -v
python -m kaixuan_control_lab.validate
python -m kaixuan_control_lab.run --controller gravity --seconds 5

# 在电脑桌面打开模型，默认运行 30 秒
python -m kaixuan_control_lab.run --controller gravity --seconds 30 --viewer
```

物理步长 1 ms。默认固定基座、左臂七轴可动、右臂固定在 URDF 零位。
初始左臂角度也为 URDF 零位，**没有读取真机当前姿态**。
`--viewer` 需要本机桌面/OpenGL；普通 SSH 终端使用无窗口命令即可。
绘制外观网格，但当前自由空间基线不计算碰撞/接触。

单独生成可查看的 MJCF：

```bash
python -m kaixuan_control_lab.model
# 输出 artifacts/left_arm.xml 和配套配置/URDF 哈希元数据
```

每次 `run` 会生成独立的 `artifacts/<UTC时间>/`：模型、配置快照、`trace.csv`、`summary.json`。
CSV 每行是时刻 t 的 q、dq，以及用于 [t,t+dt) 的请求力矩、限幅/限速后指令、执行器输出。
`summary.json` 记录耗时分位数、被限制的指令次数、最大速度、最终状态和退出原因。
模型和日志是生成文件，Git 忽略它们，拉取源码后在本地重新生成。

`--controller zero` 是无重力补偿的对照，机械臂会在仿真中下落，可能触发速度保护并以退出码 1 结束。
这里的速度保护是退出仿真实验，不等于真实驱动器的 CW6、抱闸或停止锁存。

## 加上 Pinocchio 交叉验证

```bash
python -m pip install -r requirements-pinocchio.txt
python -m kaixuan_control_lab.validate --pinocchio
```

PyPI 包名是 `pin`，Python 导入名是 `pinocchio`。可用 wheel 取决于电脑架构/Python 版本；
若当前环境已装官方 Pinocchio，也可以直接使用该环境。不要把 SSH 机器的私有路径复制到代码里。
启用 `--pinocchio` 后，缺依赖会明确失败，不会跳过检查再报告通过。

检查包含 32 个固定随机样本的 M、G、c+G、RNEA、末端位置/旋转和雅可比。
Pinocchio 与 MuJoCo 都从同一 URDF 建模，并应用同一份 armature。
通过只说明模型转换与数学约定一致；物理参数的真实性仍需实机数据验证。

## 第 7 轴惯量

配置位于 [config/left_arm.yaml](config/left_arm.yaml)。左右第 7 轴均登记了：

```yaml
inertia:
  armature_kg_m2:
    left_joint7: 0.02
    right_joint7: 0.02
```

默认只启用左臂，右侧值是同规格假设下的预留值。
`0.02 kg·m²` 是 **输出侧附加转动惯量的工程初值**，没有经过辨识。
模型使用 `M_effective = M_URDF + diag(armature)`；Pinocchio 的 RNEA 同时增加 `armature * ddq`。
不添加虚构末端负载，不用增大 URDF 球形标记质量来代替转子惯量。
URDF 本身没有通用的 armature 字段，因此直接用原始 URDF 调用 Pinocchio 并不会自动得到这个补充值。
详细推导、未知参数和后续辨识顺序见 [MODEL_ASSUMPTIONS.md](MODEL_ASSUMPTIONS.md)。

## 代码入口

| 文件 | 作用 |
| --- | --- |
| `config/left_arm.yaml` | 关节顺序、惯量估计、力矩/速度限制、执行器与物理摩擦假设 |
| `kaixuan_control_lab/model.py` | 保留 URDF 位姿/轴方向/惯量，生成固定基座 MJCF |
| `kaixuan_control_lab/interfaces.py` | `JointState`、`Dynamics`、`TorqueController` 接口 |
| `kaixuan_control_lab/backend.py` | MuJoCo 物理步进、动力学查询、限幅/延迟/响应模型 |
| `kaixuan_control_lab/pinocchio_model.py` | 可选的独立动力学与运动学实现 |
| `kaixuan_control_lab/controllers.py` | 已实现重力补偿和零力矩基线 |
| `kaixuan_control_lab/run.py` | 实验循环、可选窗口、CSV 与 JSON 日志 |
| `kaixuan_control_lab/validate.py` | 动力学、方向、单位与跨引擎数值验收 |
| `tests/test_foundation.py` | 惯量影响、重力平衡、指令限制与状态保护测试 |

统一接口：`controller.compute(state, dynamics)` 返回 **总关节力矩 Nm**。
基线中的重力已经由控制器加入；后端不会再加一次 G，也不会把 Nm 乘以 101。
MuJoCo actuator 使用 `motor gear="1"`。不要将关节速度、位置或电流当成这个接口的力矩输入。
以后由运行器管理 1 ms 物理步长和较慢的优化器周期；当前基线每个物理步都更新控制器。

## 本次验收

2026-11-05，Linux aarch64、Python 3.10.12、MuJoCo 3.3.7、NumPy 1.26.4、Pinocchio 4.0.0：

- 6 个基础测试通过；完整 STL 外观模型编译成功。
- 32 组跨引擎检查通过，结果见 [validation/baseline_report.json](validation/baseline_report.json)。
- 5 秒无窗口重力补偿保持初始静止；第 7 轴 +0.01 Nm 得到约 +0.5 rad/s²。
- 零力矩对照约 0.23 秒触发 90°/s 速度保护，非零退出并保存故障状态。
- 已验证 0.5/1/2 倍第 7 轴惯量只改变相应质量矩阵对角项，不改变重力力矩。
- 桌面窗口需在你的电脑验证；未测试真实执行器延迟/带宽，未运行真机控制。

参考：[MuJoCo 3.3.7 armature](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#body-joint-armature)、
[MuJoCo 力矩执行器](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#actuator-motor)、
[Pinocchio](https://github.com/stack-of-tasks/pinocchio)。
