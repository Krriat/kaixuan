# 左臂 CST 重力补偿

`kaixuan_can_hardware/GravityController` 是 ros2_control 的 C++ effort 控制器。
它在 controller_manager 的 1000 Hz 循环中直接读取关节状态，用 Pinocchio 计算重力项，
输出关节侧 N·m，沿用现有 EtherCAT CST、编码器偏置、方向和单位换算。
硬件的 `read()` 在同一锁内复制位置、速度和力矩快照，计算期间不直接读取 EtherCAT 线程正在写入的数组。

## 计算方法

采用固定基座模型，URDF 根坐标系为 world，重力向量为 `[0, 0, -9.80665]` m/s²。
当前 URDF 的 world 到 base_link 无旋转；实体安装与模型一致时，该向量也适用于基座。
控制器按 URDF 关节名称映射位置和速度，不依赖消息或接口的排列顺序。

Pinocchio 使用 URDF 的关节变换、轴线、质量和质心计算：

```text
g(q) = computeGeneralizedGravity(model, data, q)
tau_command = ramp * gravity_scale * g(q) - B * dq
```

动力学约定是 `M*ddq + C + g(q) = tau`，因此补偿直接使用 `+g(q)`。
`B*dq` 提供速度阻尼；控制器没有位置弹簧，目标是抵消模型重力，而非保持固定角度。
减速比和电机符号由硬件接口处理，控制器不再重复乘减速比。

前 0.5 秒只输出零力矩，供硬件完成 `6 → 7 → 15`；随后默认用 5 秒增加到目标补偿比例。
第 5 轴的归一化角度可直接用于动力学，三角函数对 ±180° 的等价表示连续。
硬件相对位移保护仍使用连续编码器角度。

## 依赖与构建

可使用系统 ROS 包 `ros-humble-pinocchio`。本机使用配套 ROS Humble 二进制包的私有副本，
不修改系统 Python、NumPy 或 ROS 安装：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
bash src/kaixuan_can_hardware/scripts/install_pinocchio.sh
colcon build --base-paths src --packages-select kaixuan_can_hardware kaixuan3_config --symlink-install
source install/setup.bash
```

私有目录默认 `~/.local/share/kaixuan/pinocchio/opt/ros/humble`，CMake 会自动查找。
自定义目录可通过 `-DKAIXUAN_PINOCCHIO_PREFIX=/目录` 指定。
缺少 Pinocchio 时，仅重力控制器不构建，现有 CSP 和普通 CST 控制器仍可构建。

## 启动、激活、停用

```bash
# 默认 mock，左臂七个关节。先加载并配置控制器，保持 inactive。
ros2 launch kaixuan3_config gravity_compensation.launch.py

# 实体左臂，默认 100% 模型重力项，激活后缓慢增加。
ros2 launch kaixuan3_config gravity_compensation.launch.py use_mock_hardware:=false
```

启动后从另一终端激活：

```bash
source /opt/ros/humble/setup.bash
source /home/agxorin/cxl/kaixuan/xiaogong_kaixuan/install/setup.bash
ros2 control switch_controllers --activate gravity_compensation_controller
```

**激活会自动发送新零命令、释放刹车并逐渐施加重力补偿。**
当前启动流程按下垂姿态验证；若启动时手臂已抬起，零力矩和缓升阶段的补偿尚未建立，
应支撑手臂直至补偿建立。此入口尚未实现从位置保持向重力补偿的带载无缝切换。
无需向 `/torque_controller/commands` 发布消息；重力控制器直接占用 effort 接口。
普通力矩控制器与重力控制器不能同时占用同一个关节的命令接口。

停用：

```bash
ros2 control switch_controllers --deactivate gravity_compensation_controller
```

退出 launch 也会停用并释放主站。硬件保护锁存后，先停用整个重力控制器，再激活，
才能重新经过零力矩准备和缓升。仅恢复正常状态不会自行重新使能。

初次核验可在启动时设置较小比例，例如 `gravity_scale:=0.25`；默认 `1.0`。
参数在控制器配置时读取，修改后需要重新启动 launch。默认关节是 left_joint1..7，
可用 `joints:=left_joint1,left_joint2,...` 显式选择。模型计算仍读取完整机器人状态，
所选关节以外的命令接口不被占用。

## 参数、负载与保护

- `gravity_config_file`：重力向量、各关节阻尼和零力矩准备时间，默认 `config/gravity_compensation.yaml`。
- `gravity_ramp_time`：缓升秒数，默认 5，允许 1–60 秒。
- `torque_config_file`：硬件限幅和保护，重力入口默认 `config/gravity_limits.yaml`。
- `payload_mass`：URDF 之外附加的工具/工件质量，kg，默认 0，避免与模型质量重复计算。
- `payload_frame`：负载参考连杆坐标系，填 URDF 连杆名，默认 left_ee。
- `payload_com`：负载质心相对该坐标系的位置，逗号分隔的米制坐标，例如 `0,0,0.05`。

例如，在模型之外添加 0.5 kg、质心位于工具坐标系 Z 方向 5 cm 的负载：

```bash
ros2 launch kaixuan3_config gravity_compensation.launch.py use_mock_hardware:=false \
  payload_mass:=0.5 payload_com:=0,0,0.05
```

重力入口默认力矩上限为额定输出力矩的 20%，变化率为额定值的 10%/秒；
速度上限为 45°/秒（0.7853981633974483 rad/s），相对使能位置的位移范围为每轴 120°，另有模型绝对位置保护。
120° 是相对本次准备位置的范围，部分关节可能先达到 URDF 绝对位置限位。
与普通 CST 入口一样，可在启动时通过 `max_velocity_rad_s`、`max_displacement_deg`
显式覆盖拖动速度和相对位移上限。未指定时仍使用 YAML 的限值；修改后需重启 launch。
计算所需力矩超过限值时，控制器报错并使整组命令失效，不会默默截断补偿。
无效反馈、超过 50 ms 的控制器更新间隔同样停用；硬件另有独立 100 ms write 看门狗。

本入口设置 `stop_cst_group_on_fault=true`：一个已占用关节出现硬件保护，
全部已占用 CST 关节撤力并锁存停用；保护码 10 表示跟随其他关节的故障停用。
原始故障轴保留自己的保护码。普通 CST 入口仍沿用按轴保护配置。
保护发生后，模型控制器可能继续显示 active 并计算力矩，但硬件已将实际目标置零、控制字置 6。
后台诊断线程现在会打印一次具体保护原因与触发时数值，诊断话题也会锁存原始现场。

2026-11-04 用户拖动时“补偿一段时间后消失”的现场：第 1 轴保护码为 8（当时为 10° 相对位移限值），
其他六轴为 10（整组联动停用），实际目标力矩全部为零，驱动错误码均为 0。
随后按用户要求将相对位移扩大到 120°，速度仍为 45°/秒。

## 查看计算结果

```bash
ros2 topic echo /gravity_compensation_controller/gravity_torques
ros2 topic echo /gravity_compensation_controller/commanded_torques
```

两者均为 Float64MultiArray，按 `joints` 顺序排列，20 Hz 发布。
前者是未乘比例的 Pinocchio `g(q)`，后者是缓升、比例和阻尼计算后的指令。
实际下发的限速后力矩、原始目标、使能状态和保护码以 `/kaixuan/hardware_diagnostics` 为准。

URDF 几何对齐不等于质量、质心和电机力矩常数已经标定。
当前 N·m 仍按额定参数换算；摩擦、线缆、减速机效率以及未建模负载会留下补偿误差。
mock 仅用于模型计算和控制器生命周期联调，不模拟重力下的机械响应。

第 3、5 轴若在当前姿态下的重力项很小，单纯重力补偿不会明显减轻水平转动的阻力。
应先确认电机使能且刹车已释放，再区分传动摩擦、刹车拖滞和速度阻尼。
本控制器没有摩擦辨识或摩擦前馈；不能通过放大重力系数来代替它们。

## 2026-11-04 验证记录

- 构建通过，Pinocchio 版本 4.0.0。
- 14 项 CTest 与 10 项左臂标定检查通过。
- 解析单摆验证重力力矩符号、大小、竖直姿态、反向安装和附加负载质心。
- 左右臂共 50 组离线姿态中，Pinocchio 与独立 KDL 重力计算最大差约 `1.6e-14 N·m`，
  与势能数值梯度最大差约 `9.87e-9 N·m`。
- mock 验证默认 inactive、激活后的新零命令、缓升至完整模型力矩、effort 接口输出及停止/重启。
- 实体只测试左臂当前下垂姿态：25% 比例稳态 5 秒、100% 比例稳态 30 秒，均另含零力矩准备、
  5 秒缓升、整组停用和重启。100% 测试最大位移约 `0.00717°`，运行 WKC 异常增量为 0，
  无 DC 失锁、驱动报错或保护触发。
- 完整模型输出的第 1、2、4 轴平均指令约 `-0.1873、-0.1120、-0.0778 N·m`。
  驱动反馈仍为额定参数估算，在此小力矩下存在量化与跟踪误差；本次没有外部扭矩标定，
  也未验证全姿态拖动效果或负载变化。
- 测试后全部轴未使能；左臂目标力矩、速度为零，无故障，原 PDO/模式已恢复；两个主站 Idle。
  右臂控制设置与测试前一致。

原始诊断、模型输出、测试脚本、停机核对与汇总：
`/home/agxorin/ec_generic/logs/ros2-gravity-left-arm-20261104T021540Z/`，主汇总为 `results.json`。
目录时间为 UTC。

后续 120° 范围更新：重新构建后，14 项 CTest 与 10 项标定检查通过；
实机保持未激活，确认七轴实际加载 45°/秒、120°，控制字均为 6、零目标力矩、无驱动故障。
这次没有重新使能或执行 120° 运动。保护快照新增了首因锁存测试，并覆盖 119° 允许、121° 停用。
URDF 结合实时编码器还原的当前第 4、6 轴约为 29.91°、23.52°；第 3、5 轴的模型重力项均约 -0.027 N·m。
姿态图、核对结果和构建记录位于
`/home/agxorin/ec_generic/logs/cst-guard120-pose-20261104T031933Z/`。

算法参考：[Pinocchio computeGeneralizedGravity](https://github.com/stack-of-tasks/pinocchio/blob/devel/include/pinocchio/algorithm/rnea.hpp)。
