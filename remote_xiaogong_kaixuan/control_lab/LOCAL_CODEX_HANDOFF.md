# 给电脑端 Codex 的交接

## 用户的目标

用户已经在真实左臂实现 CST 力矩控制、重力补偿和部分低速摩擦辅助。
现在要通过实践掌握最优化、数值优化、LQR、MPC、WBC 和常见 QP 求解方法。
先在电脑端 MuJoCo 完成每一阶段，再选择合适实验部署机器人。希望慢慢实现、理解推导，
不是一次生成所有控制器，也不是仅调用现成框架而跳过建模。

当前工作目录应是本地仓库的 `remote_xiaogong_kaixuan/control_lab`。
这是本地电脑代码目录，来自 SSH 机器上的同名仓库；不能硬编码 `/home/agxorin/...`。
新仿真模块没有 ROS/网络/电机通信路径，原来的 RViz/MoveIt 客户端保持独立。

## 先读这些文件

1. `AGENTS.md`：此目录的约定。
2. `README.md`：已有能力与本机复核命令。
3. `MODEL_ASSUMPTIONS.md`：第 7 轴惯量、参数可信度、后续辨识。
4. `ROADMAP.md`：概念关系、各阶段模型/产出/验收。
5. `config/left_arm.yaml`、`kaixuan_control_lab/interfaces.py`：具体参数和代码边界。

## 已有基础，避免重复搭建

- 当前 URDF 已反映用户认可的几何和轴方向。第 6 轴 local +Z，第 2 轴 local -Z。
- 没有末端工具。左右第 7 轴输出侧 armature 初值均 0.02 kg·m²，默认仿真只活动左臂。
- armature 是估计值，非厂家数据/辨识结果。模型假设以文件为准，不要悄悄变成“已知真实参数”。
- MJCF 生成器保留 URDF 的显式惯量；不按视觉 STL 推算质量。
- 默认固定基座、右臂 q=0、左臂七自由度、1 ms 物理步长、Nm 输入且 gear=1。
- 基础运行器有重力/零力矩基线、限幅、可选斜率/延迟/一阶响应、日志和状态保护。
- 独立 Pinocchio 适配器使用同一 armature；已验证 M/G/c+G/FK/J/RNEA 的一致性。
- 阶段 01 二维 QP 已实现，入口 `python -m kaixuan_control_lab.run_qp2d`；
  推导和验收见 `experiments/01_qp/`，不要重复实现。
- 阶段 02 已按用户要求替换为七轴末端直线跟踪与短时外力扰动，入口
  `python -m kaixuan_control_lab.run_lqr --pinocchio`；三种控制器、PD 扫描、较快 PD、延迟和保护记录见 `experiments/02_lqr/`。
- 原第七轴单独回零实验的配置、控制器、测试、说明、生成结果已删除；不要恢复该学习路线。
- `joint_velocity_qp/` 在仓库根目录已有 Eigen+OSQP C++ 关节速度 QP，阶段 04 可复用。

初次完成记录：2026-11-05，Linux aarch64、Python 3.10.12、MuJoCo 3.3.7、NumPy 1.26.4、
Pinocchio 4.0.0；6 项测试、32 组动力学交叉验证、完整网格编译和 5 秒重力基线通过。
该次没有桌面 viewer 或真机动力学验证。

本机复核与阶段 01：2026-11-06，Linux x86_64、Python 3.10.12、MuJoCo 3.3.7、
Pinocchio 4.0.0、NumPy 2.2.6、OSQP 1.1.3；6 项基础测试、32 组跨引擎检查、
5 秒重力基线和 8 秒桌面 viewer 通过。8 项 QP 测试与 235 个样本通过，
独立参考误差约 6.35e-9。该轮环境随后按用户要求迁移至 `mujoco_test` Conda 环境，旧 `.venv` 已删除；
真实轴方向仍需用户人工确认。

阶段 02 重做：Conda `mujoco_test`，Python 3.10.20。七轴全部可动，末端固定姿态走直线。
末端实际原点受世界系推力，通过 J^T F 施加到 qfrc_applied；不叠入电机命令，控制器不接收推力。
每 10 ms 更新，1 ms 物理步；控制器输出总 Nm，重力只加一次。
时变 LQR 离线设计；与参考前馈加独立 PD、计算力矩加 PD 对比，扫描 8–80 rad/s。
用户要求看清运动与扰动，默认配置已改为 `config/cartesian_lqr_demo.yaml`：
30 cm/5 秒运动、仿真 3.00–4.50 秒峰值 5 N 侧推，回放 0.5 倍速并加中文提示与实时曲线。
小扰动基准保留在 `config/cartesian_lqr.yaml`；强扰动演示只要求 LQR 常规完成，其余保护停机照实记录。
新演示结果与观看说明在 `experiments/02_lqr/VISIBLE_DEMO.md`。
较快计算力矩 PD 在理想模型中可取得更小误差，必须保留该结果，不能只展示较慢 PD。
验收数字与完整命令以 `experiments/02_lqr/VALIDATION.md` 为准；尚未实现 MPC 或真机控制。

## 后续协作

先解释并查看当前末端轨迹/误差、推力、耦合增益、强 PD 对照与保护停机。
用户希望一步一步理解，不自动进入 MPC/WBC，也不连接真实机器人。
若扩展当前实验，统一参考与限制，保留失败记录并明确模型假设。

## 交付边界

本次改动保存在电脑工作区；**其他机器只有在提交并推送后，才能通过 `git pull` 取得源码**。
交接文件本身不表示已推送。Git 操作时核对原有未提交模型/规划改动，避免把其他任务误打包。
`artifacts/` 和 Python 环境是本地生成文件，不应提交；重建定义 `environment.yml` 可以提交。
如果电脑端只有整个仓库的一个子目录副本，本目录依赖 `../model/`，需要一起同步。
