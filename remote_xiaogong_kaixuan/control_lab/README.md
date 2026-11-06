# 凯旋力矩控制实验室

这是电脑端的 **离线 MuJoCo 实践环境**，目标是逐步掌握 QP、LQR、MPC、WBC。
模型取自 `../model/robot.urdf`，不依赖机器人上的安装路径、ROS 或 EtherCAT。
全部命令从本目录执行。模型、参数、算法和实验日志统一采用关节输出轴的 SI 单位。

已经实现阶段 00：URDF → MJCF、七轴力矩执行器、仿真状态接口、重力/零力矩基线、
可选执行器延迟与一阶响应、日志、独立动力学交叉验证。
阶段 01 二维 QP 也已实现：OSQP、独立活跃集参考解、KKT 验收、失败案例和教学 ADMM。
阶段 02 已替换为七轴末端直线跟踪：时变 LQR、两种 PD 对照、末端外力、延迟与增益扫描。
原第七轴单独回零实验及其结果已删除。MPC/WBC 留作后续，具体见 [ROADMAP.md](ROADMAP.md)。
给本地 Codex 的上下文和第一条任务见 [LOCAL_CODEX_HANDOFF.md](LOCAL_CODEX_HANDOFF.md)。

## 电脑端启动

本机已创建名为 `mujoco_test` 的 Conda 环境，后续直接激活：

```bash
cd remote_xiaogong_kaixuan/control_lab
conda activate mujoco_test

# 无窗口的数值检查和 5 秒重力补偿基线
python -m unittest discover -s tests -v
python -m kaixuan_control_lab.validate
python -m kaixuan_control_lab.run --controller gravity --seconds 5

# 在电脑桌面打开模型，默认运行 30 秒
python -m kaixuan_control_lab.run --controller gravity --seconds 30 --viewer
```

新机器首次安装时，从本目录执行 `conda env create -f environment.yml`，再激活环境。
该文件包含阶段 00/01/02 和 Pinocchio 所需依赖，并设置 `PYTHONNOUSERSITE=1`，
防止 `~/.local` 的 Python 包覆盖环境内版本。本项目旧 `.venv` 已在迁移和验证后移除。

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

## 阶段 01：二维 QP

在本目录的 Python 环境中增加本阶段依赖，然后运行：

```bash
python -m pip install -r requirements-qp.txt
python -m unittest discover -s experiments/01_qp/tests -v
python -m kaixuan_control_lab.run_qp2d
```

这是独立的无量纲数值实验，不需要启动机械臂模型。运行后在 `artifacts/qp2d_<UTC时间>/`
生成可行域/等高线、目标移动与活跃约束、ADMM 收敛三张图，以及 CSV、失败案例和验收摘要。
推导、矩阵形式、图表含义和验收标准见 [阶段 01 说明](experiments/01_qp/README.md)，
本机结果见 [阶段 01 验收](experiments/01_qp/VALIDATION.md)。基础测试和 QP 测试独立，
只安装 `requirements.txt` 仍可运行原来的阶段 00。

2026-11-06 本机复核：Python 3.10.12、MuJoCo 3.3.7、Pinocchio 4.0.0；
6 项基础测试、32 组跨引擎验证、5 秒重力基线和 8 秒桌面 viewer 通过。
8 项 QP 测试及 235 个实验样本通过，参考解最大误差约 `6.35e-9`。
窗口启动通过不等于真实关节方向已由用户确认；未进行真机控制。

## 阶段 02：末端直线运动与外力扰动

```bash
conda activate mujoco_test
python -m unittest discover -s experiments/02_lqr/tests -v
python -m kaixuan_control_lab.run_lqr --pinocchio
# 完成数值实验后，打开真实记录的 MuJoCo 回放
python -m kaixuan_control_lab.run_lqr --pinocchio --viewer
```

默认使用 `config/cartesian_lqr_demo.yaml`：左臂七轴全部可动，末端用 5 秒走约 30 cm
直线并保持姿态；在仿真 3.00–4.50 秒施加世界 +Y 方向推力，峰值 5 N。
较强扰动使部分 PD 对照触发原有保护，完成要求明确设为 LQR；停机记录全部保留。
原 7.48 cm、0.5 N、0.12 s 的小扰动基准保存在 `config/cartesian_lqr.yaml`，可显式选择：
`python -m kaixuan_control_lab.run_lqr --config config/cartesian_lqr.yaml --pinocchio`。
控制器不接收推力信息。
比较“参考动力学前馈＋独立 PD”“计算力矩＋PD”“时变 LQR”，并扫描 PD 增益，
包含较快的计算力矩 PD 对照，不预设 LQR 获胜。
演示输出参考轨迹、800 个反馈增益、CSV/NPZ、比较图和明确的保护停机记录。
加 `--video` 生成中文讲解视频：0.5 倍速、侧推大箭头、实时推力和位置误差、路径投影。
观看顺序和结果见 [强扰动演示](experiments/02_lqr/VISIBLE_DEMO.md)。
推导、复现、回放与解释见 [阶段 02](experiments/02_lqr/README.md)，数据见 [本机验收](experiments/02_lqr/VALIDATION.md)。

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
| `config/cartesian_lqr_demo.yaml` | 默认 30 cm、5 N、1.5 s 强扰动演示 |
| `config/cartesian_lqr.yaml` | 原小扰动基准、权重、PD 频率和验收参数 |
| `kaixuan_control_lab/trajectory.py` | 保持姿态的直线、连续逆运动学、关节参考样条 |
| `kaixuan_control_lab/lqr.py` | 七轴线性化、ZOH、有限时域 Riccati、三种总力矩控制器 |
| `kaixuan_control_lab/run_lqr.py` | 实验、交叉验证、增益扫描和汇总入口 |
| `kaixuan_control_lab/lqr_replay.py` | 从记录回放 MuJoCo，支持窗口与 MP4 |
| `kaixuan_control_lab/lqr_video.py` | 中文阶段提示、同步力/误差曲线、等比例路径投影 |
| `kaixuan_control_lab/run.py` | 实验循环、可选窗口、CSV 与 JSON 日志 |
| `kaixuan_control_lab/validate.py` | 动力学、方向、单位与跨引擎数值验收 |
| `tests/test_foundation.py` | 惯量影响、重力平衡、指令限制与状态保护测试 |

统一接口：`controller.compute(state, dynamics)` 返回 **总关节力矩 Nm**。
基线中的重力已经由控制器加入；后端不会再加一次 G，也不会把 Nm 乘以 101。
MuJoCo actuator 使用 `motor gear="1"`。不要将关节速度、位置或电流当成这个接口的力矩输入。
重力/零力矩基线每个物理步更新控制器；阶段 02 的独立运行器实现 1 ms 物理步长与
10 ms 控制采样保持。LQR 增益离线计算，在线只计算反馈，不运行优化求解器。

## 初始验收

2026-11-05，Linux aarch64、Python 3.10.12、MuJoCo 3.3.7、NumPy 1.26.4、Pinocchio 4.0.0：

- 6 个基础测试通过；完整 STL 外观模型编译成功。
- 32 组跨引擎检查通过，结果见 [validation/baseline_report.json](validation/baseline_report.json)。
- 5 秒无窗口重力补偿保持初始静止；第 7 轴 +0.01 Nm 得到约 +0.5 rad/s²。
- 零力矩对照约 0.23 秒触发 90°/s 速度保护，非零退出并保存故障状态。
- 已验证 0.5/1/2 倍第 7 轴惯量只改变相应质量矩阵对角项，不改变重力力矩。
- 该次未测试桌面窗口或真实执行器延迟/带宽，未运行真机控制；本机复核结果见上文。

参考：[MuJoCo 3.3.7 armature](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#body-joint-armature)、
[MuJoCo 力矩执行器](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#actuator-motor)、
[Pinocchio](https://github.com/stack-of-tasks/pinocchio)。
