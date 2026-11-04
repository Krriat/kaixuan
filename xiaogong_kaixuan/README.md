# 小工凯旋 ROS 2 工作区

此目录以 `../` 的凯旋工作区为基础，使用 `/home/agxorin/cxl/xiaogong-kaixuan` 的 URDF 和网格，模型包放在 `src/xiaogong_kaixuan`。`kaixuan3_config` 的模拟控制与 MoveIt 配置，以及 `kaixuan_can_hardware` 的 URDF 引用，均已切换到这个模型。源工作区和源模型目录未修改。

模型有左右臂各 7 个活动关节；关节名与原配置相同。新的关节轴向、机械尺寸和限位来自小工模型。MoveIt 的 SRDF 保留规划组和零位状态，只保留相邻连杆的碰撞豁免。

左臂编码器补偿位于 `src/kaixuan3_config/launch/moveit_bringup.launch.py` 的 `_left_comp_defaults`。
2026-10-29 根据保持不变的实体姿态读取七个编码器，以 RViz 截图的 `[-180, -180, -7, 39, 180, -17, -6]°` 为目标角度重新计算。
±180° 按 URDF 实际限位 ±3.14 rad 处理。原始读数、公式及新旧补偿见 `src/kaixuan3_config/config/left_arm_calibration.json`。
URDF 几何已恢复为 `2d714d5` 版本，机器人端重启 bringup 后使用新的编码器补偿。

在此目录中构建并启动模拟配置：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch kaixuan3_config moveit_bringup.launch.py use_real_hardware:=false
```

此目录里的 `COLCON_IGNORE` 避免在上层凯旋工作区构建时发现同名包；因此在此目录构建时须使用 `--base-paths src`。

## CST 力矩模式

现有 `ros2_control` 硬件插件已增加 CST：选定关节使用 `effort` 命令，
`/joint_states.effort` 提供按额定参数换算的输出侧 N·m。原位置控制默认仍为 CSP。
使用前述环境加载步骤后，可启动单轴真实硬件：

```bash
ros2 launch kaixuan3_config torque_control.launch.py \
  use_mock_hardware:=false joints:=left_joint7
```

`left_joint7` 对应 Master1 / slave6；省略 `use_mock_hardware:=false` 时使用 mock。
启动后收到新零力矩命令才执行 `6 → 7 → 15`，默认指令超时 200 ms 后撤力、下使能并锁存。
使能会释放刹车，零力矩不保持位置。关节选择、持续发令、限幅和恢复流程见
[CST 使用说明](src/kaixuan_can_hardware/TORQUE_CONTROL.md)。

左臂第 5 轴现使用 `left_comp4=274.56318430°`，反馈归一化到 `[-180°, 180°)`；
原 `180.018°` 表示为约 `-179.982°`，对应同一物理姿态。该轴 URDF 与力矩限位均为精确的 ±π。

已增加基于 Pinocchio 的重力补偿控制器，按 `g(q) - B*dq` 输出关节力矩。
启动左臂真实硬件（控制器先保持 inactive）：

```bash
ros2 launch kaixuan3_config gravity_compensation.launch.py use_mock_hardware:=false
```

再执行 `ros2 control switch_controllers --activate gravity_compensation_controller`，
控制器先以零力矩使能，再用 5 秒增加到完整模型补偿。
停用使用 `--deactivate gravity_compensation_controller`。
依赖安装、模型参数、附加负载、限值和验证记录见
[重力补偿说明](src/kaixuan_can_hardware/GRAVITY_COMPENSATION.md)。

## I210 / ec_igb 启动时序修复（2026-11-03）

### 当前默认配置：双臂参考时钟跟随

已把本工作区的 MoveIt launch、硬件 launch、真实硬件 xacro 和 C++ 缺省值统一为
**reference 模式、1 ms 周期、500 us 相位**。默认 `active_arm=both`：
右臂使用 I210 Master0 / CPU6，左臂使用 I210 Master1 / CPU7，各连接七个驱动。
每条总线独立跟随其第一个从站的时钟。保留启动资格、CSP 准备、失锁保护与停机禁用流程。
当前默认组合不包含 O6；`host` 模式仍可显式选择。

已使用下面这条直接 launch 命令完成双臂同时 600 秒测试，未覆盖时钟、相位、CPU 或诊断参数，
未启用额外的 RT 参考时钟取证，也未发送运动轨迹。日志核验实际加载了上述默认值。

| 总线 | 驱动数 | 启动厂家计数增量 | 600 秒运行厂家计数增量 | WKC 异常 | 主站 Lost |
| --- | --- | --- | --- | --- | --- |
| Master0 / 右臂 | 7 | 全 0 | `[0,0,0,0,0,0,0]` | 0 | 0 → 0 |
| Master1 / 左臂 | 7 | 全 0 | `[0,0,0,0,0,0,0]` | 0 | 2 → 2，未新增 |

每站均有 121 次厂家计数读数，完整覆盖 0–600 秒；厂家计数未清零。
测试后 14 个驱动均为 CW6、无故障、PREOP，两个主站 Idle，原监控选择器已恢复。
时钟寄存器仍为原值 4096 / 12，控制、测试和采样进程均已退出。
这是一轮双臂静止验证，尚未覆盖运动和更长时间负载。
`move_group` 在硬件完成 shutdown 后仍有已知的退出析构 -11，单独记录，未在本次修复。

原始记录和核验结果：`~/ec_generic/logs/dual-i210-reference-defaults600-20261103T102432Z/`，
其中 `summary.json` 为测试结果，`verified-results.json` 为逐主站计数、参数和位置变化核对。
默认值修改前的备份：`~/ec_generic/workspace-backups/dual-reference-defaults-20261103T102204Z/`。

直接启动（真实驱动会使能保持当前位置）：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaixuan3_config moveit_bringup.launch.py \
  use_real_hardware:=true use_rviz:=false
```

也可以使用 `bash ~/ec_generic/run_i210_moveit.sh`，默认核验并启动双臂。
单臂可追加 `active_arm:=left` 或 `active_arm:=right`；`--check` 只核验、不使能。
Ctrl-C 执行停用并退出。默认模拟模式仍需显式设置 `use_real_hardware:=true` 才连接实体。

### 此前左臂单独实测：参考从站领时 + 更快的有界跟随

本机 I210 / ec_igb、Master1、左臂七轴、MoveIt 启动并使能保持当前位置，未发送运动轨迹。
以下均使用 **1 ms 周期、500 us 同步相位、reference 模式**；厂家计数没有清零。

| 独立启动的测试 | 启动厂家计数增量 | 运行厂家计数增量（七站） | 运行 WKC 异常 | 主站 Lost 新增 |
| --- | --- | --- | --- | --- |
| 300 秒 | 全 0 | `[0,0,0,0,0,0,0]` | 0 | 0 |
| 重启后 600 秒 | 全 0 | `[0,0,0,0,0,0,0]` | 0 | 0 |

两轮均未触发跟随失锁，停机后七轴未使能、无故障，主站释放回 Idle、从站回 PREOP。
历史同端口原版启动最大 +174，更早另一 I210 端口为 +594；此前“配置后重置 DC”方案的
600 秒复测最大 +424、七站合计 +2208。这些是不同轮次对照，不能据此保证所有工况永久零丢帧。
目前结论限于两轮静止测试，尚未覆盖运动、其他端口、泛用网卡或更长时间负载。

在当前接线下复现这组参数：

```bash
bash ~/ec_generic/run_i210_moveit.sh \
  active_arm:=left dc_clock_mode:=reference shift_time_ns:=500000 \
  dc_reset_after_configuration:=false
```

该命令会使能并保持当前位置，Ctrl-C 停用退出；上述两轮记录仅为 Master1 / left、七轴无 O6。
这两轮测试时默认仍为 `host` / 100 us；当前已改为前述双臂新默认。单独复现左臂时需追加
`active_arm:=left`。源码已合入并构建在本工作区，7 项 CTest 通过。

两轮原始记录分别在：

- `~/ec_generic/logs/i210-startup-fix-runtime-reference-fast300-20261103T094632Z/`
- `~/ec_generic/logs/i210-startup-fix-runtime-reference-fast-repeat600-20261103T095437Z/`

两轮汇总：`~/ec_generic/logs/reference-fast-two-runs.txt`（JSON 同名）；每轮目录中的
`fast-profile-analysis.json` 保存计数、实际加载参数、时序统计和停机核验。

目录时间采用 UTC。两轮均启用固定容量的参考时钟取证缓冲及低优先级逐站寄存器采样；
逐周期缓冲只保留最后 8192 拍，完整窗口的厂家计数、WKC 统计和周期诊断另行保存。
MoveIt 的 `move_group` 在退出析构时仍出现独立的 -11 崩溃；硬件停用和释放已完成并单独核验，
这不是运行窗口的丢帧，也尚未在本次工作中修复。

### 先前启动修复与失败试验记录

已将 `~/ec_generic/i210_startup_fix` 中实测的启动门控、CSP 准备阶段、实际应用时间及停机流程
合入本工作区，重新构建 `kaixuan_can_hardware` 和 `kaixuan3_config`。
直接加载本工作区即可使用修复版，无需再加载临时修复工作区。
现有标定、关节方向、减速比和控制器配置保留；默认仍为模拟模式，真实硬件须显式指定。

**后续复测发现稳态计数仍可能增长：不能将此前两轮零增量理解为已经完全消除丢帧。**
合入原工作区后，首次默认参数实测启动增量仍为全站 0，随后 60 秒增量为
`[228, 384, 140, 104, 254, 150, 178]`，最大 +384；WKC 和主站 Lost frames 均无异常，
退出无 Er300。同一时段 DC 同步监测上界升至 524287 ns，但尚未证明二者的因果关系。
随后复测临时修复库，启动与稳态再次为零。核心循环代码、编译和链接选项、有效参数已核对；
尚不能把差异归因于代码合入，也不能宣称间歇稳态问题已经修复。
记录分别为 `~/ec_generic/logs/i210-startup-fix-integrated-original-defaults-20261103T062735Z/`
和 `~/ec_generic/logs/i210-startup-fix-overlay-repeat-after-integration-20261103T063032Z/`。

原工作区第二次复测延长到 120 秒，启动仍全站 0，稳态后段又出现
`[70, 36, 88, 74, 64, 52, 24]`，最大 +88；WKC 与主站 Lost frames 仍无异常，退出无 Er300。
记录为 `~/ec_generic/logs/i210-startup-fix-integrated-original-repeat120-20261103T063352Z/`。
合入及残留问题汇总为 `~/ec_generic/logs/integrate-original-results-20261103.json`。
目前可确认启动顺序和停机行为已改善，不能确认运行中的间歇厂家计数增长已经消除。

当前接线为 I210 Master 1，7 个驱动。保持当前位置启动命令：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaixuan3_config moveit_bringup.launch.py \
  use_real_hardware:=true active_arm:=left use_rviz:=false
```

此命令启动真实驱动并使能保持当前位置，Ctrl-C 停用并退出；不会主动发送运动轨迹，
但运行中的控制器可以接受其他 ROS 客户端的运动目标。left 对应 Master 1，right 对应 Master 0；
未传 active_arm 时仍默认 both，当前只有一条从站链，应显式传 left。

默认时序为 `cycle_time_ns:=1000000`、`shift_time_ns:=100000`、`priority:=90`、
`master1_affinity:=7`。`record_diagnostics` 和 `bus_diagnostics` 默认 false，可显式开启；
硬件周期采样与状态发布仍保留。单独硬件 bringup、Xacro 和 C++ 缺省相位也统一为 100000 ns。

已验证的组合包括全部 OP / 完整 WKC / DC 监测合格之后再使能、100 个 CSP 准备周期、
实际唤醒时间作为应用时间、1 ms 周期、100 us 相位，以及先停用再释放总线。
临时修复版两轮测试的启动与各 60 秒稳态厂家计数均为零增量，退出均无 Er300；
原版当前端口启动最大 +174、稳态最大 +218，历史另一 I210 端口启动最大 +594。
构建使用与试验版一致的 RelWithDebInfo（`-O2 -g`）；各项修改的贡献没有逐项隔离。
这里的结果针对静止保持，尚未验证运动、长时间运行和双主站。

2026-11-03 的加长复测确认：启动改善仍然有效，但上述短测零增长不能代表稳态问题已经解决。
调整参考校时时间戳、同步频率、SYNC0 相位、参考站滤波和带宽，均未得到稳定重复的零增长。
“配置完成后用原值重新初始化参考站时钟调节器，再禁用观察 10 秒”曾在 300 秒中全站零增长，
同条件重启复测 600 秒却增加 `[108,286,416,424,396,248,330]`，单站最大 +424。
该可选参数 `dc_reset_after_configuration` 因此保持默认 false；参数原值始终为 4096 / 12，
厂家计数没有清零。完整成功、失败及提前停止的试验记录保存在
`~/ec_generic/logs/experiments-summary.json` 和同名 `.txt` 中。

多数厂家计数增长时，PDO 的 WKC 完整、主站 Lost 没有增加，所以不能直接把厂家计数等同于
网卡或线路实际丢失的以太网帧。实测从站 DC 时钟仍有较大的周期性偏差；具体厂家判据未公开。
`dc_clock_mode:=reference` 现为默认模式，支持 right、left 或 both，各主站须为 1 ms、七轴无 O6；
可显式设置 `dc_clock_mode:=host` 返回主机领时模式。实验模式让 EtherCAT 线程跟随参考站时钟，不调整 Linux 系统时钟，
也不向参考站发送主机校时 FPWR；每周期的 PDO 和从站间 FRMW 同步仍保持。

### 参考时钟跟随实验的含义

`host` 模式由 AGX 的时钟带领从站；`reference` 模式以第一个从站的时钟为准，
AGX 微调下一拍的唤醒时间跟随它，不修改 Linux 系统时钟。仍按参考时钟每 1 ms 通信一次。
当前默认使用 500 us 的 SYNC0 相位；先前的 host 对照使用过 100 us。相位改变不等于周期改变。

参考时钟读数属于上一拍发送的同步报文，必须与保存的上一拍发送时间配对。
拿本拍时间比较，会人为多出约 1 ms 的误差。实现保留了回绕、读数有效性和周期对应检查。
参考站不再接收主机校时时，其 `0x092C` 可能保留旧值，不能拿它代表 AGX 与参考站的误差；
该误差由上述配对时间戳计算，其他从站的 `0x092C` 则用于观察从站间的同步。

实测旧跟随参数遇到约 124 ppm 的参考时间速率变化时响应过慢；同一时段
`CLOCK_MONOTONIC` 相对 `CLOCK_MONOTONIC_RAW` 的速率没有对应突变。
这还不能区分参考站自身速率变化与未测量的发送到线上延迟，不能直接断言晶振故障。
本轮候选将比例/积分系数改为 `0.03 / 0.000075`，保持 `1/32` 滤波，
粗调进入细调的误差条件收紧至 60 us。提高这些系数是加快跟随响应，并非提高通信频率。

粗调转细调时初始化积分状态，避免控制量突然改变；若积分限幅，仍由每拍最大 5 ns 的
变化限制保证有界平滑交接。每拍时间修正的 ±1000 ns 限幅、10 秒未使能观察和原有失锁保护保持不变；累计时差补偿可以超过毫秒。
积分诊断值在交接时的跳变是算法初始化，不能解释为从站实际频率跳变。
离线测试已覆盖启动、变速、短脉冲及保护锁存；密集变速和持续强噪声仍可能触发保护，
是否可用于长期运行必须以完整实机复测为准。

### 三个时间量分别控制什么

以下按先前 `host` 模式和 100 us 相位说明，用于解释时间量；当前默认为 reference / 500 us。`reference` 模式使用实际唤醒单调时间 `M`
减去累计补偿 `B` 作为应用时间 `A = M - B`；参考时间网格 `P` 每拍增加 1 ms，
实际 Linux 唤醒截止时间为 `P + B`。它不执行下面的主机到参考站校时操作。

`1 ms = 1000 us = 1000000 ns`，`100 us = 0.1 ms = 100000 ns`。

| 时间量 | 代码 / 参数 | 意义 |
| --- | --- | --- |
| 本次实际唤醒时间 | `steady_now_ns()` → `ecrt_master_application_time()` | 告诉 IgH 本次循环开始时，AGX 的单调时钟已经走到哪里 |
| 周期间隔 | `cycle_time_ns = 1000000` | 周期线程的目标间隔和驱动 SYNC0 间隔，均为 1 ms |
| 同步相位 | `shift_time_ns = 100000` | SYNC0 相对周期参考点后移 100 us，事件间隔仍为 1 ms |

循环时间变量 `cycle_time` 在 C++ 中表示本次采样的绝对时间戳，`cycle_time_ns_` 表示周期长度，
两者不能混用。时间戳来自 `CLOCK_MONOTONIC`，与终端显示的日期、时区无关。

这里需要区分**周期相位参考**和**发给参考从站的校时时间戳**。目前循环入口把实际唤醒时间
传给 `ecrt_master_application_time()`；首次应用时间还会被 IgH 保存为 `dc_ref_time`，
用于建立 SYNC0 的周期网格。之后应用时间变化不会每拍重新建立这个网格。

原版把计划唤醒时间传给应用时间接口。计划在 10.000 ms 醒来、实际在 10.025 ms 执行时，
计划时间与实际时间相差 25 us。但不能简单说“使用计划时间就是错误”：IgH 官方 `dc_user`
示例本身也使用计划时间建立周期相位，同时另取最新实际时间校正参考从站。
需要避免的是将较早的时间戳用于较晚才排队发送的时钟校正，而没有考虑这段执行延迟。

当前新增 `dc_reference_time` 参数用于对照测试：

- `cycle_start`：调用 `ecrt_master_sync_reference_clock()`，沿用本拍入口的应用时间校时。
- `before_sync`：处理完收包和 PDO 后，再读取 `CLOCK_MONOTONIC`，调用
  `ecrt_master_sync_reference_clock_to(master, now)`。应用时间和已有 SYNC0 周期网格保持原逻辑。

本机选 Slave 0 的 DC 时钟为参考。`host` 模式每周期先排队参考时钟校正，再调用
`ecrt_master_sync_slave_clocks()` 排队从站间同步，最后随 `ecrt_master_send()` 发出。
参考站跟随主机提供的时间，其他从站跟随参考站；这是时钟控制过程，不是每次报文都立即硬跳时钟。
I210 在这条实现中传输 EtherCAT 帧，程序没有使用 I210 的 PTP 硬件时钟作为 DC 参考。

新增加的 `reference_age_at_send` 观测“采样校时时间戳 → 调用发送函数”的间隔。
它不包含完整的网卡排队、物理出线和线路传播延迟，也不是硬件时间戳。
实际时间采样不会使 Linux 自动准时唤醒；减少软件中的时间戳陈旧程度与消除所有丢帧是两回事。

周期线程原本就用 `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` 按绝对截止时间运行，
`host` 模式每轮把下一次计划唤醒时间增加 1 ms。正常情况下它每秒调用约 1000 次收发；这不代表每轮只有一帧，
也不代表驱动内部电流环或速度环必定为 1 kHz。
本次新增 `ecrt_master_set_send_interval(master, cycle_time_ns_ / 1000)`，传入 1000，
因为该 API 的单位是 us。它告诉 IgH 应用预计的发送间隔，供状态机报文预算及相关调度使用。
实际周期 PDO 发送仍由应用线程调用 `ecrt_master_send()` 完成，该 API 不会自动启动一个 1 kHz 发包器。

从站有本地 DC 硬件时钟，SYNC0 是依据此时钟产生的周期同步事件，驱动固件依此安排周期工作。
本机调用 `ecrt_slave_config_dc(..., 1000000, 100000, 0, 0)` 设置其周期和相位。
以 IgH 建立的周期参考点为示意零点：

| 第几拍 | 周期参考点 | 相位为 0 的 SYNC0 | 相位为 100 us 的 SYNC0 |
| --- | --- | --- | --- |
| 第 1 拍 | 0 us | 0 us | 100 us |
| 第 2 拍 | 1000 us | 1000 us | 1100 us |
| 第 3 拍 | 2000 us | 2000 us | 2100 us |

三次 SYNC0 之间仍相隔 1000 us，不会累加成 1100 us 的通信周期。
这个参考点由 IgH 的应用时间与 DC 启动配置确定；表中不是抓包测得的真实发包时刻。
100 us 也不是“每帧发出去后等待 100 us”，程序没有为此增加 sleep。

调相位的目的，是改善“主站准备并送达 PDO”与“从站按本地节拍使用数据”的相对时序。
相位不合适时，帧即使收到了、WKC 完整，也可能未赶上驱动该拍的数据使用时刻。
100 us 是先前启动改善测试使用的参数，稳态仍出现过厂家计数增长，不能认为它为每一帧保证了 100 us 的完整余量：
调度晚到、执行时间、传播时间和驱动固件内部的取数安排都会影响实际余量。
具体是哪项机制使厂家监控项 64 增长，目前尚无厂家固件判据或直接相位测量可以定论。

此外，启动时的“100 个 CSP 准备周期”按 1 ms 计算约为 100 ms，与 100 us 的 SYNC0 相位是两回事。

API 语义核对：[IgH 应用接口文档](https://docs.etherlab.org/ethercat/1.6/doxygen/group__ApplicationInterface.html)。
本机代码为 `src/kaixuan_can_hardware/src/kaixuan_can_system.cpp` 中的 `configure_master`、
`configure_axis`、`cyclic_loop`；本机 IgH 1.6.10 源码位于 `~/ec_generic/src/igh-ethercat-1.6.10/`。

重建与检查：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select kaixuan_can_hardware kaixuan3_config \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
ctest --test-dir build/kaixuan_can_hardware --output-on-failure
```

合入前源码和旧库备份位于
`~/ec_generic/workspace-backups/integrate-startup-fix-20261103T062443Z/`。

小工 URDF 的 14 个活动关节均写有 `effort=0`、`velocity=0`。现有 `joint_limits.yaml` 为 MoveIt 规划另设 1 rad/s 的速度上限；这不是小工实体硬件的额定参数。复制的 EtherCAT 电机映射、编码器方向、减速比、零位补偿及实际碰撞行为尚未按小工机械臂校准，不能直接用于实体运动。`src/normal` 和 `src/kaixuan_bam` 中的原凯旋脚本也未做硬件适配。
