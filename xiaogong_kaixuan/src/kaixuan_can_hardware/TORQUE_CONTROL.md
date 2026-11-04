# CST 力矩控制

`KaixuanCanSystem` 支持启动时选择 `control_mode=csp|cst`。现有 MoveIt / bringup
仍使用 CSP；CST 由独立 launch 加载同一个硬件插件，向选定关节导出 `effort`
命令接口，并向 `/joint_states` 提供 `position`、`velocity`、`effort` 反馈。
模式选择在硬件启动时完成；切回 CSP 请先退出 CST launch，再启动原位置控制。
基于模型的自动力矩输出另见 [重力补偿说明](GRAVITY_COMPENSATION.md)。

## 启动

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select kaixuan_can_hardware kaixuan3_config --symlink-install
source install/setup.bash

# 默认使用 mock 硬件，可检查控制器、话题和接口。
ros2 launch kaixuan3_config torque_control.launch.py joints:=left_joint7

# 连接实体：left_joint7 对应 Master1 / slave6。
ros2 launch kaixuan3_config torque_control.launch.py \
  use_mock_hardware:=false joints:=left_joint7
```

不带参数的 `ros2 launch kaixuan3_config torque_control.launch.py` 默认选择
`use_mock_hardware=true`、`joints=left_joint7`。它读取模型、左臂标定和力矩限值，
启动 `ros2_control_node`（controller_manager，1000 Hz）及 `robot_state_publisher`，
再由 spawner 加载并激活 `joint_state_broadcaster` 和 `torque_controller`。
状态广播器发布 `/joint_states`，模型状态发布器发布 TF；力矩控制器等待
`/torque_controller/commands`。mock 模式使用 `mock_components/GenericSystem`，不请求 EtherCAT 主站。

launch 根据所选关节决定使用左臂、右臂或双臂主站。多个关节使用逗号分隔，
例如 `joints:=left_joint6,left_joint7`，命令数组严格按这个顺序排列。
所选总线仍交换全部七轴 PDO；CST 只为选中的关节导出命令接口并允许使能。
左臂偏置、方向及减速比默认读取 `kaixuan3_config/config/left_arm_calibration.json`；
右臂偏置默认与当前 MoveIt launch 一致。均可使用 `comp0..6`、`left_comp0..6`、
`left_direction1..7`、`right_direction1..7`、`*_gear_ratio1..7` 参数覆盖。

只启动 launch 时关节保持未使能。控制器激活、总线/DC 就绪，且收到新的零力矩
命令后，硬件才执行 `6 → 7 → 15` 使能驱动；零力矩不提供位置保持。
机械抱闸是否存在、如何随使能动作，需要核对实际型号与接线，不能仅由控制字判断。

## 命令与停用

控制器为 `kaixuan_can_hardware/TorqueController`，输入是
`/torque_controller/commands`，消息类型 `std_msgs/msg/Float64MultiArray`，单位为
**输出关节 N·m**，正方向与 ROS 关节正方向一致，减速比不会再乘一次。

单轴示例，先持续发送零力矩以完成使能：

```bash
ros2 topic pub -r 50 /torque_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.0]}'
```

实际控制程序应由**同一个持续运行的发布器**完成零力矩准备、后续力矩命令和撤力。
默认超过 200 ms 未收到命令，控制器会使接口失效，硬件立即写入零目标力矩并下使能。
不要用间隔较长的单次命令拼接持续运动。停止上面的发布器也会触发此超时停用。

另外可以显式停止控制器：

```bash
ros2 control switch_controllers --deactivate torque_controller
```

超时、异常命令、失锁、通信中断、模式改变、运行故障、越限均锁存停用；恢复发布
不会自行重新使能。重新启动控制器并发送新的零力矩指令，才可重新准备：

```bash
ros2 control switch_controllers --deactivate torque_controller
ros2 control switch_controllers --activate torque_controller
```

退出 launch 会撤力、下使能并释放主站。再次启动 CSP 时，其原 PDO 配置会重新下发。
硬件保护按关节处理；一个关节越限不会自动停用其他关节。话题超时或消息长度/
数值无效时，控制器会使它管理的全部关节命令失效。
重力补偿入口会另外启用 `stop_cst_group_on_fault=true`，将硬件保护扩展到全部已占用 CST 关节。

## 参数与换算

`config/torque_limits.yaml` 按关节设置：

| 参数 | 含义 |
| --- | --- |
| `rated_torque_nm` | 输出侧额定力矩，用于反馈换算 |
| `torque_command_units_per_nm` | 每 N·m 对应的 `0x6071` 原始单位，可标定 |
| `effort_limit_nm` | 指令绝对值上限，默认额定力矩的 5% |
| `effort_slew_nm_s` | 指令变化率上限 |
| `max_velocity_rad_s` | 转速保护，默认 45°/秒，即 0.7853981633974483 rad/s |
| `max_displacement_rad` | 从本次零命令准备位置起的位移保护，默认 120° |
| `min_position_rad` / `max_position_rad` | 绝对关节位置保护，需要与标定和 URDF 一致 |

可通过 `torque_config_file:=/绝对路径/config.yaml` 加载自己的参数。默认位移范围
用于初次调试，扩大运行范围时应按应用修改相应限值。超过指令上限会限幅；
位置、速度或位移越限会撤力并锁存停用。

当前默认 `max_velocity_rad_s=0.7853981633974483`，即 45°/秒，默认相对位移为 120°（2.0943951023931953 rad）。
此前初始调试限值为 0.3 rad/s（约 17.2°/秒），用户轻推时触发超速保护后，按用户要求调整为 45°/秒。
初始 10° 位移限值在后续拖动中触发整组停用，现按用户要求扩大到每轴相对使能位置 120°。
保护码 7 是速度超限，8 是位移超限；触发后目标力矩为零、控制字变为 6，下使能，
不是驱动自动切回位置模式。CW6 不证明机械抱闸锁住负载；恢复发令不会自行清除锁存。

启动时也可显式指定速度，例如：

```bash
ros2 launch kaixuan3_config torque_control.launch.py use_mock_hardware:=false \
  joints:=left_joint1,left_joint2,left_joint3,left_joint4,left_joint5,left_joint6,left_joint7 \
  max_velocity_rad_s:=0.7853981633974483
```

这将速度上限设置为 45°/秒。若要单独调整相对位移范围，可设置 `max_displacement_deg`。
两项均为空时沿用 YAML；它们在启动时生效，需要退出原 launch 再启动，运行实例不会热更新。
指令超时、硬件 write 看门狗、驱动故障、模式检查及绝对位置限位仍然有效。
启动零力矩前应托住手臂，并持续以 50 Hz 发布七个零值；停止发布超过 200 ms 会撤力并下使能。
实测第 2 轴在 CW6、零目标力矩下仍移动，机械抱闸未确认时，撤力后仍需可靠外部支撑。
保护锁存后，可停用再激活 `torque_controller` 并重新持续发布零命令，或重启 launch。

2026-11-04 用户轻推后再次停住的现场只读诊断：left_joint1、left_joint4 的
`torque_guard_fault=7`，其余轴为 1（无效/超时命令）；所有轴均 CW6、驱动错误码 0、
模式 10，且读取时零力矩话题没有发布器。已确认发生软件保护停用；当次触发的瞬时速度
没有被旧程序保存，不能从停住后的零速度反推峰值。

`left_joint5` 的偏置 `left_comp4` 已从 `-85.43681570°` 加一整圈变为
`274.56318430°`；同一参考编码器位置对应 `-180°`。该轴反馈通过
`q = (q_unwrapped + π) mod (2π) - π` 归一化到 `[-180°, 180°)`，
原来的 `180.018°` 会显示为约 `-179.982°`。MoveIt 与 CST 使用同一偏置。
本地和 remote URDF、力矩限位的第 5 轴范围统一为精确的 `[-π, π]`。
位置命令会选择最接近当前实际编码器读数的等价目标，保持编码器圈数；
相对位移保护使用连续编码器角度，跨过 ±180° 时不会误报约 360° 的位移。

默认 `0x6071` 换算采用额定力矩千分比，对应 KDE 固件 `0x00050012` 的左臂七轴
CST 小力矩结果。V1.2 手册的 CANopen PT 示例将该对象写为 0.01 A，不能直接套用于
所有 CST 固件。其他固件或电机需核对并设置 `torque_command_units_per_nm`。
N·m 数值是按额定参数换算的估计值，尚未用外部扭矩传感器标定。

硬件配置时读取 `Pn083`、`Pn085`、`Pn088`，不修改它们：

- `Pn083=0`：`0x6077` 按额定力矩千分比换算；`Pn083=1`：按 0.01 A、额定电流和额定力矩换算。
- `Pn088=0`：速度按电机 count/s 换算；`Pn088=1`：按电机 rpm 换算。
- 所选 CST 关节还必须通过 `0x6502` bit 9 的支持检查，并在周期反馈中确认 `0x6061=10`。

指令话题超时由 `command_timeout` 设置，单位秒，范围 0.02–1.0。
硬件另有 `effort_write_timeout_ms`（默认 100 ms），用于发现 controller_manager
停止调用 `write()`；EtherCAT 周期线程独立执行撤力和停用。

## 诊断

`/kaixuan/hardware_diagnostics` 的每轴字段包括 `effort_nm`、`commanded_effort_nm`、
`target_torque_raw`、`actual_mode`、`effort_claimed`、`torque_armed`、`torque_guard_fault`。
`actual_mode` 对 CST 来自 `0x6061`；CSP 沿用请求模式值。

保护触发后，后台诊断线程会在终端打印一次具体原因、触发速度/位移及其限值。
周期线程只保存定长快照，不执行日志 I/O。相关字段包括：

- `guard_fault_snapshot_valid`：是否存在本轴直接触发保护的现场；整组联动停用的轴不伪造现场。
- `guard_fault_position_rad`、`guard_fault_anchor_rad`、`guard_fault_displacement_rad`：触发位置、连续角度参考及相对位移。
- `guard_fault_velocity_rad_s`、`guard_fault_command_nm`：触发时的速度和输入命令。
- `max_velocity_rad_s`、`max_displacement_rad`：当前实例实际加载的保护阈值。

快照锁存首个保护现场，不会被停住后的零速度或后续指令超时覆盖；重新准备时清除。
控制器显示 active 只表示软件在计算，保护后实际出力应以本话题的控制字和原始目标为准。

保护码：0 正常；1 无效命令（含话题超时）；2 硬件 write 超时；3 通信/DC；
4 模式不符；5 驱动故障/意外退出使能；6 位置；7 速度；8 相对位移；9 无效反馈；10 组内其他轴故障。
发生保护时目标力矩立即为零，控制字为 6。初次准备时只允许复位通信故障
Er300/Er400；出力后的驱动故障会锁存，避免自动反复重启。

mock 硬件只用于 ROS 接口联调，不模拟减速机、重力、刹车或硬件保护逻辑。

## 验证记录

已完成构建、11 项 CTest、10 项左臂标定测试，以及 mock 下的 effort 接口和正负命令传递。
2026-11-03 在 Master1 / slave6 上，通过 ROS 控制器发送 0、+0.075、-0.075 N·m，
确认实际模式为 10，原始目标为 0、+5、-5，最大位移约 0.024°。
实测覆盖无命令不使能、新零命令使能、停止发布后撤力并下使能、异常锁存、
恢复非零命令不自动重启、重启控制器后以新零命令重新准备、显式停用和正常退出。

用户随后允许整个左臂测试，按关节单独启动并验证；每次只有当前测试关节允许使能。
下表保留第 5 轴归一化修改前的测试记录：

| 关节 | Master1 从站 | 正负测试指令 | 最大位置变化 | 结果 |
| --- | --- | --- | --- | --- |
| left_joint1 | slave0 | ±0.340 N·m | 0°（编码器读数无变化） | 通过 |
| left_joint2 | slave1 | ±0.340 N·m | 0.00004° | 通过 |
| left_joint3 | slave2 | ±0.200 N·m | 0°（编码器读数无变化） | 通过 |
| left_joint4 | slave3 | ±0.200 N·m | 0°（编码器读数无变化） | 通过 |
| left_joint5 | slave4 | 零命令及小正命令均被拦截 | 0° | 位置保护码 6，未使能 |
| left_joint6 | slave5 | ±0.075 N·m | 0.0214° | 通过 |
| left_joint7 | slave6 | ±0.075 N·m | 0.0241° | 通过 |

六个出力轴的正负反馈方向均正确，均通过超时、锁存、重启和显式停用验证。
各轮就绪后的运行窗口内 WKC 异常计数未增长，无 DC 失锁或驱动报错。
结束后两个主站均 Idle，14 轴均 CW6、零目标力矩、零速度、无故障；
左臂 PDO 与模式已恢复测试前配置（CSP=8）。测试过程没有使能右臂。
完整诊断、关节状态、测试脚本和汇总保存在
`/home/agxorin/ec_generic/logs/ros2-cst-left-arm-20261103T123020Z/`，汇总文件为 `results.json`。

第 5 轴归一化后已单独补测：初始 ROS 反馈约 -179.9816°，0、±0.075 N·m 指令
以及超时停用、锁存、控制器重启全部通过，最大位移约 0.0996°，无驱动故障或运行 WKC 异常。
补测结束后的角度约为 -179.8927°，14 轴均已下使能、零目标力矩、无故障，
PDO 和驱动模式恢复测试前配置。归一化边界两侧的位置回算及连续位移保护由自动测试覆盖。
补测记录为 `/home/agxorin/ec_generic/logs/ros2-cst-joint5-normalized-20261103T124703Z/`。

读取的左臂姿态在新角度表示下约为 `[0, 0, 0, 0, -179.982°, 0, 0]`，当前 URDF 正运动学计算
肩部到末端方向距基座坐标系的竖直向下约 0.28°，与用户确认的下垂姿态一致。
这些是小力矩和停用行为测试，未验证负载运动、力控精度或整臂重力补偿。
