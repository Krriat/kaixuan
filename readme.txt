LinkerHand O6 EtherCAT-CAN Gateway 使用说明
============================================

1. 当前系统配置
---------------

O6 可接入两个现有 EtherCAT 主站，不使用独立主站：

  Master0 Position 0~6：原有 7 个电机从站
  Master0 Position 7：右手 LinkerHand O6 EtherCAT-CAN Gateway（可选）
  Master1 Position 0~6：原有 7 个电机从站
  Master1 Position 7：左手 LinkerHand O6 EtherCAT-CAN Gateway（可选）

每个主站周期为 1 ms，各自复用原有 Domain、DC 同步和实时线程。O6 使用 DC Sync0，
SM2 Process Data Watchdog 保持启用。主站停止发送 PDO 约 100 ms 后，从站硬件
Watchdog 仍可能使 O6 失能。

右手保持已有 ROS 接口 `/linkerhand_o6/...`；左手使用独立接口
`/linkerhand_o6_left/...`。两只手的 enable、目标、状态和压力数据完全独立。
要同时启用两只手，启动参数须为 `active_arm:=both`；仅启用左手时使用
`active_arm:=left`。

当前默认是“保持使能直通模式”：

  o6_max_step=255
  o6_command_timeout_ms=0

收到一个新目标后，主站只增加一次 CommandSequence，并在每个 1 ms PDO 周期中
持续写入相同目标、相同序号和 ControlWord=1。测试脚本退出不会自动失能。


快速操作流程
------------

1. 第一个终端启动右臂和灵巧手主站：

  source /opt/ros/humble/setup.bash
  source /home/agxorin/cxl/kaixuan/install/setup.bash
  ros2 launch kaixuan3_config moveit_bringup.launch.py \
    use_real_hardware:=true use_rviz:=false active_arm:=right

2. 第二个终端加载环境，并确认灵巧手在线：

  source /opt/ros/humble/setup.bash
  source /home/agxorin/cxl/kaixuan/install/setup.bash
  ros2 topic echo --once /linkerhand_o6/online

正常应显示 data: true。

3. 读取六个手指关节的当前位置：

  ros2 topic echo --once /linkerhand_o6/actual_position

数组顺序为：

  [拇指弯曲, 拇指横摆, 食指弯曲, 中指弯曲, 无名指弯曲, 小指弯曲]

4. 在当前位置使能，不主动产生动作：

  python3 /home/agxorin/cxl/kaixuan/src/normal/test_linkerhand_o6.py --baseline
  ros2 topic echo --once /linkerhand_o6/ready

正常应显示 data: true。

5. 发送运动目标。目标是六个 0～255 的绝对位置；第一次测试只把一个关节修改
5 个单位。例如当前位置为 [254, 255, 254, 254, 254, 254]，让食指小幅运动：

  ros2 topic pub --once /linkerhand_o6/target_position \
    std_msgs/msg/UInt8MultiArray \
    "{data: [254, 255, 249, 254, 254, 254]}"

发送前必须以实际读取的位置为准，不要直接照抄示例中的数值。

自动小幅运动并测量延迟，可以运行：

  python3 /home/agxorin/cxl/kaixuan/src/normal/measure_linkerhand_o6_latency.py

默认测试食指当前位置减5，打印指令传输延迟、实际响应延迟和到位时间，停留0.5秒后
返回原位置。如果脚本原先执行了使能，测试结束后会自动失能。查看可选参数：

  python3 /home/agxorin/cxl/kaixuan/src/normal/measure_linkerhand_o6_latency.py --help

让六个位置通道同时持续伺服往返，并验证压力扫描持续运行：

  python3 /home/agxorin/cxl/kaixuan/src/normal/exercise_linkerhand_o6_with_pressure.py

默认幅度100、以50 Hz连续发送不同的位置目标，并一直循环到按 Ctrl+C。主站的1 ms
循环始终保持压力 RequestMask bit2。脚本每2秒打印位置命令接受数、CAN TX/RX帧率、
估算总线负载和压力序号。停止后平滑返回原位置；如果由脚本执行了使能，随后自动失能。
调整幅度和发送频率：

  python3 /home/agxorin/cxl/kaixuan/src/normal/exercise_linkerhand_o6_with_pressure.py \
    --amplitude 150 --rate 100

负载率默认按1 Mbps CAN、每帧约120个线上比特估算。该值用于趋势观察，精确负载率
应以CAN分析仪或固件底层总线统计为准。

python3 src/normal/test_linkerhand_o6.py --namespace /linkerhand_o6_left
python3 src/normal/measure_linkerhand_o6_latency.py --namespace /linkerhand_o6_left
python3 src/normal/exercise_linkerhand_o6_with_pressure.py \
  --namespace /linkerhand_o6_left

6. 读取压力传感器状态和200个压力点：

  ros2 topic echo --once /linkerhand_o6/pressure_valid
  ros2 topic echo --once /linkerhand_o6/pressure_status
  ros2 topic echo --once /linkerhand_o6/pressure

正常情况下 pressure_valid 为 true。pressure_status 数组为：

  [PressureSequence, FingerValidMask, PressureState, PressureAgeMs]

其中 FingerValidMask 应为31（0x1F），PressureState 应为2，PressureAgeMs 应小于500，
PressureSequence 连续观察时应持续递增。pressure 中单点索引为：

  finger * 40 + row * 4 + column

finger 为0～4（大拇指到小拇指），row 为0～9，column 为0～3。

7. 使用结束后显式失能：

  ros2 service call /linkerhand_o6/enable \
    std_srvs/srv/SetBool "{data: false}"


2. 编译
-------

修改代码后执行：

  cd /home/agxorin/cxl/kaixuan
  source /opt/ros/humble/setup.bash
  colcon build --packages-select tilt_kaixuan kaixuan_can_hardware kaixuan3_config

编译完成后，每个新终端都要加载环境：

  source /opt/ros/humble/setup.bash
  source /home/agxorin/cxl/kaixuan/install/setup.bash


3. 启动前检查
-------------

禁止同时启动两个 EtherCAT/ROS 主站。先检查：

  pgrep -af 'ros2_control_node|moveit_bringup.launch.py'

如果已经有旧 launch，应在旧 launch 所在终端按 Ctrl+C，等待它完整退出后再启动。
停止主站会释放机械臂电机和 O6，请先支撑好机械臂。


4. 启动主站
-----------

O6 位于 Master0，因此启动右臂硬件：

  source /opt/ros/humble/setup.bash
  source /home/agxorin/cxl/kaixuan/install/setup.bash

  ros2 launch kaixuan3_config moveit_bringup.launch.py \
    use_real_hardware:=true use_rviz:=false active_arm:=right

在另一个终端检查 EtherCAT：

  sudo ethercat slaves

正常情况下 Master0 的 Position 0~7 应全部为 OP，其中 Position 7 显示：

  LinkerHand O6 EtherCAT-CAN Gateway

还可以检查 Domain：

  sudo ethercat domains -v -m 0

正常 WorkingCounter 应为 24/24。


5. 查看 O6 状态
-------------

  ros2 topic echo --once /linkerhand_o6/online
  ros2 topic echo --once /linkerhand_o6/ready
  ros2 topic echo --once /linkerhand_o6/actual_position
  ros2 topic echo --once /linkerhand_o6/temperature
  ros2 topic echo --once /linkerhand_o6/joint_error
  ros2 topic echo --once /linkerhand_o6/pressure_status
  ros2 topic echo --once /linkerhand_o6/pressure_valid
  ros2 topic echo --once /linkerhand_o6/status
  ros2 topic echo --once /linkerhand_o6/realtime_stats

最常用的当前位置查询命令是：

  ros2 topic echo --once /linkerhand_o6/actual_position

输出的 data 数组就是六个关节当前位置，顺序与目标位置完全相同：

  [拇指弯曲, 拇指横摆, 食指弯曲, 中指弯曲, 无名指弯曲, 小指弯曲]

需要连续观察位置变化时，去掉 --once：

  ros2 topic echo /linkerhand_o6/actual_position

查看位置反馈发布频率：

  ros2 topic hz /linkerhand_o6/actual_position

查看所有 O6 话题和消息类型：

  ros2 topic list -t | grep /linkerhand_o6

查看目标话题的发布者和订阅者：

  ros2 topic info -v /linkerhand_o6/target_position

/linkerhand_o6/temperature 返回六个电机温度值，关节顺序与位置相同。

/linkerhand_o6/joint_error 返回六个关节错误码。正常应全部为 0。

/linkerhand_o6/pressure 返回 200 个 uint8 压力点。数组索引为：

  finger * 40 + row * 4 + column

其中 finger 为 0～4（大拇指到小拇指），row 为 0～9，column 为 0～3。

/linkerhand_o6/pressure_status 的数组顺序：

  [0] PressureSequence，应持续递增
  [1] FingerValidMask，完整五指为 0x1F
  [2] PressureState，完整数据为 2
  [3] PressureAgeMs

/linkerhand_o6/pressure_valid 在 FingerValidMask=0x1F、PressureState=2 且
PressureAgeMs<500 时为 true。

/linkerhand_o6/online 表示 EtherCAT OP、Domain Working Counter 和 O6 在线位是否正常。

/linkerhand_o6/ready 表示当前位置使能握手已完成，且 EtherCAT OP/WC、
O6 在线位、位置/错误反馈和全部关节错误码都满足安全条件。发生关节错误
（StatusWord bit4=1 或 joint_error 非零）时会立即变为 false；错误清除后，握手状态仍为
Ready 时会自动恢复为 true。只有 ready=true 才接受目标话题。

/linkerhand_o6/status 的数组顺序：

  [0]  StatusWord
  [1]  AppliedCommandSequence
  [2]  PositionSequence
  [3]  主站 CommandSequence
  [4]  ValidMask
  [5]  ActiveCanId，正常为 39，也就是 0x27
  [6]  CanTxCount
  [7]  CanRxCount
  [8]  CanErrorCount
  [9]  O6TimeoutCount
  [10] HandshakeState

StatusWord 位定义：

  bit0  0x0001：O6 在线
  bit1  0x0002：历史上出现过 CAN 错误
  bit2  0x0004：历史上出现过 O6 超时
  bit3  0x0008：历史上出现过 CAN 接收溢出
  bit4  0x0010：当前存在关节错误

ValidMask 位定义：

  bit0  0x01：位置有效
  bit1  0x02：温度有效
  bit2  0x04：关节错误信息有效
  bit3  0x08：压力有效

/linkerhand_o6/realtime_stats 的数组顺序：

  [0] CycleCount，1 ms 实时循环总周期数
  [1] QueueCount，调用 ecrt_domain_queue 的次数
  [2] SendCount，调用 ecrt_master_send 的次数
  [3] WorkingCounter，当前 Domain WKC，正常为 24
  [4] WcState，正常 COMPLETE 为 2
  [5] AlState，正常 OP 为 8
  [6] MaxCycleLatenessNs，最大周期唤醒延迟，单位 ns
  [7] MaxLoopIntervalNs，1 ms 循环最大实际间隔，单位 ns
  [8] LastSendTimeNs，最近一次发送时间，CLOCK_MONOTONIC ns
  [9] LastCompleteSendTimeNs，最近一次 WKC COMPLETE 的发送时间

正常运行时应满足：

  WorkingCounter=24
  WcState=2
  AlState=8
  CycleCount、QueueCount、SendCount 持续增加且彼此只相差约 1

HandshakeState：

  0：Disabled
  1：EnableBaseline
  2：WaitBaselineAck
  3：Ready
  4：WaitCommandAck

接受运动目标前必须满足：

  online=true
  ready=true
  HandshakeState=3


6. 首次使能并保持当前位置
-----------------------

推荐使用：

  python3 /home/agxorin/cxl/kaixuan/src/normal/test_linkerhand_o6.py --baseline

这个命令会：

  1. 读取 ActualPosition[6]。
  2. 调用 /linkerhand_o6/enable=true。
  3. 先发送至少 20 个 ControlWord=0 周期。
  4. 把当前 ActualPosition 作为基线目标。
  5. 发送 ControlWord=1，并使用新序号提交相同位置。
  6. 等待 AppliedCommandSequence 确认。
  7. 脚本退出，但 O6 继续保持使能和最后目标。

确认握手完成：

  ros2 topic echo --once /linkerhand_o6/ready

预期结果：

  data: true

也可以直接调用使能服务：

  ros2 service call /linkerhand_o6/enable \
    std_srvs/srv/SetBool "{data: true}"

调用后仍然要等待 ready=true 才能发送目标。


7. 发送六关节目标位置
-------------------

目标话题：

  /linkerhand_o6/target_position

消息类型：

  std_msgs/msg/UInt8MultiArray

六个数的顺序：

  0：拇指弯曲
  1：拇指横摆
  2：食指弯曲
  3：中指弯曲
  4：无名指弯曲
  5：小指弯曲

每个目标范围为 0~255，目标是绝对位置，不是增量。发送前先读取当前位置：

  ros2 topic echo --once /linkerhand_o6/actual_position

示例，发送一次六关节目标：

  ros2 topic pub --once /linkerhand_o6/target_position \
    std_msgs/msg/UInt8MultiArray \
    "{data: [252, 255, 220, 250, 246, 237]}"

首次测试建议只修改一个关节 5 个单位。虽然主站允许 0~255 全范围目标，但一次把
多个关节从接近 255 改到 0 或 100 会产生明显的大幅动作。

相同的目标重复发布不会增加 CommandSequence。主站会在 1 ms PDO 中持续保持同一
目标和同一序号，STM32 不会因为重复 PDO 而重复发送位置 CAN 帧。

AppliedCommandSequence 等于 CommandSequence 只表示 STM32 已经把命令提交给 CAN
邮箱，不代表所有关节已经到位。到位需要比较 ActualPosition 和目标值。


8. 小幅动作测试
-------------

只监视，不使能、不动作：

  python3 /home/agxorin/cxl/kaixuan/src/normal/test_linkerhand_o6.py

当前位置握手，不故意动作，退出后保持使能：

  python3 /home/agxorin/cxl/kaixuan/src/normal/test_linkerhand_o6.py --baseline

食指减少 5 后恢复原值，退出后保持使能：

  python3 /home/agxorin/cxl/kaixuan/src/normal/test_linkerhand_o6.py \
    --baseline --exercise-index

需要脚本退出时显式失能，可增加：

  --disable-on-exit


9. 显式失能
-----------

  ros2 service call /linkerhand_o6/enable \
    std_srvs/srv/SetBool "{data: false}"

此时实时线程持续发送 ControlWord=0。

关闭整个主站时，在 launch 终端按 Ctrl+C。硬件插件会先发送至少 20 个禁用周期，
然后停止 EtherCAT 循环。


10. PDO 字节检查
---------------

  sudo ethercat domains -v -m 0

只显示 O6 的 SM2/SM3 区域，可以使用：

  sudo ethercat domains -v -m 0 | grep -A20 'SlaveConfig 0:7, SM2'

找到下面这一段：

  SlaveConfig 0:7, SM2 (Output), Size 32

前 32 字节含义：

  0~1   ControlWord，小端；使能时应为 01 00
  2~3   CommandSequence，小端
  4     UpdateMask，必须为 01
  5     RequestMask，应为 00
  6~7   Reserved，应为 00 00
  8~13  TargetPosition[6]
  14~19 TorqueLimit[6]，当前位置模式写 0
  20~25 SpeedLimit[6]，当前位置模式写 0
  26~31 Acceleration[6]，当前位置模式写 0

例如下面的输出：

  01 00 71 00 01 00 00 00 fc ff 64 fa f6 ed ...

解释为：

  ControlWord=1
  CommandSequence=0x0071=113
  UpdateMask=1
  TargetPosition=[252, 255, 100, 250, 246, 237]

当前主站没有单独发布“已锁存目标位置”ROS话题。要查询主站正在保持的目标，使用
上面的 ethercat domains 命令，读取 SM2 的第 8~13 字节。直接 echo
/linkerhand_o6/target_position 只能看到仍在线的ROS发布者新发出的消息，不能可靠
查询主站内部已经锁存的最后目标。

找到下面这一段：

  SlaveConfig 0:7, SM3 (Input), Size 240

前 32 字节含义：

  0~1   StatusWord
  2~3   AppliedCommandSequence
  4~5   PositionSequence
  6     ValidMask
  7     ActiveCanId，正常为 27 十六进制，也就是十进制 39
  8~13  ActualPosition[6]
  14~19 Temperature[6]
  20~25 JointError[6]
  26~27 CanTxCount
  28~29 CanRxCount
  30    CanErrorCount
  31    O6TimeoutCount

压力区 32~239 当前不用于控制判断。

当前位置控制只使用 UpdateMask=0x01。当前没有 TorqueLimit、SpeedLimit 和
Acceleration 的 ROS 设置接口，实时线程会把这三个数组全部写成 0。它们可以在
SM2 原始数据中查看，但当前不能通过 ROS 单独设置或查询。


10.1 EtherCAT/O6 查询命令总表
----------------------------

查看两个主站总体状态、链路、帧计数和 DC：

  sudo ethercat master

只查看 Master0：

  sudo ethercat master --master 0

查看所有从站状态：

  sudo ethercat slaves

查看 Master0 的 O6：

  sudo ethercat slaves --master 0

查看 O6 PDO 映射：

  sudo ethercat pdos --master 0 --position 7

查看 O6 的 IgH C 结构定义：

  sudo ethercat cstruct --master 0 --position 7

导出 O6 ESI/XML：

  sudo ethercat xml --master 0 --position 7

查看 Domain、WKC和当前全部PDO字节：

  sudo ethercat domains --verbose --master 0

查看 O6 SM2 配置、硬件 Watchdog 和 AL 状态寄存器：

  sudo ethercat reg_read --master 0 --position 7 0x0810 8
  sudo ethercat reg_read --master 0 --position 7 0x0400 2
  sudo ethercat reg_read --master 0 --position 7 0x0420 2
  sudo ethercat reg_read --master 0 --position 7 0x0440 4
  sudo ethercat reg_read --master 0 --position 7 0x0442 2
  sudo ethercat reg_read --master 0 --position 7 0x0130 2
  sudo ethercat reg_read --master 0 --position 7 0x0134 2

当前预期：

  0x0810 返回的第 5 字节为 64，表示 SM2 Control Byte=0x64
  0x0400 为 c2 09，对应 Watchdog Divider=2498
  0x0420 为 e8 03，对应 Watchdog Time=1000
  0x0440 的 bit0 为 1
  0x0130 的 AL State 为 08，也就是 OP
  0x0134 的 AL Status Code 为 00 00

查看当前 launch 可设置的所有参数：

  ros2 launch kaixuan3_config moveit_bringup.launch.py --show-args

只查看 O6 两个启动参数：

  ros2 launch kaixuan3_config moveit_bringup.launch.py --show-args | \
    grep -A3 'o6_'

当前默认值应为：

  o6_command_timeout_ms=0
  o6_max_step=255

查看最新 ros2_control_node 日志：

  ls -t /home/agxorin/.ros/log/ros2_control_node_*.log | head -n 1

实时查看最新日志中的目标接受/拒绝信息：

  tail -f "$(ls -t /home/agxorin/.ros/log/ros2_control_node_*.log | head -n 1)"


11. 常见问题
------------

问题：发布目标后没有动作。

检查：

  ros2 topic echo --once /linkerhand_o6/online
  ros2 topic echo --once /linkerhand_o6/ready
  ros2 topic echo --once /linkerhand_o6/status

如果 ready=false，目标会被拒绝。先执行 --baseline，等待 ready=true。

问题：一直显示 Waiting for at least 1 matching subscription(s)。

说明 O6 ROS 接口尚未出现，检查主站是否启动、Position 7 是否存在且是否为 OP。

问题：日志出现 ecrt_request_master(0) failed。

通常是另一个旧 ros2_control_node 仍占用 Master0。检查：

  pgrep -af 'ros2_control_node|moveit_bringup.launch.py'

只保留一个 launch。

问题：到达位置后手指仍然感觉松。

先用 sudo ethercat domains -v -m 0 确认 O6 输出仍是 ControlWord=1、目标和序号
没有变化，并确认 AppliedCommandSequence 等于主站 CommandSequence。主站持续保持
PDO 目标不等于 O6 电机一定具有预期保持力；实际保持力还取决于 O6 CAN 固件、内部
位置环和扭矩配置。当前 UpdateMask=0x01，只更新位置，TorqueLimit/SpeedLimit/
Acceleration 均写 0。AppliedCommandSequence 也不表示机械已到位或正在输出力矩。

问题：StatusWord 为 0x0007。

bit0 表示 O6 在线；bit1、bit2 表示历史上出现过 CAN 错误或 O6 超时。继续观察
CanErrorCount 和 O6TimeoutCount 是否增长。计数不增长时可能只是历史锁存状态。


12. 安全注意事项
----------------

1. 不要同时运行两个 EtherCAT 主站 launch。
2. 停止 launch 前支撑好机械臂，停止 PDO 后机械臂和 O6 可能失去保持。
3. 首次动作不要发送六个 0，也不要同时给多个手指发送大跨度目标。
4. 直通模式没有 ROS 命令超时，最后目标会一直保持，直到显式 enable=false 或
   主站关闭。
5. EtherCAT 通信真正停止时，约 100 ms 的从站硬件 Watchdog 仍可能使 O6 失能。




ros2 topic echo --once /linkerhand_o6/online
ros2 topic echo --once /linkerhand_o6/ready
ros2 topic echo --once /linkerhand_o6/status
ros2 topic info /linkerhand_o6/target_position --verbose

watch ... ros2 topic echo --once ...
ros2 topic echo /linkerhand_o6/joint_error
ros2 topic echo /linkerhand_o6/temperature

1：过压/欠压
2：磁编码异常
4：电机过温
8：电流过流
32：负载过载
