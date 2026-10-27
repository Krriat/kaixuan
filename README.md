# Kaixuan 双臂工作空间

## 目录用途

| 路径 | 用途 |
| --- | --- |
| `src/tilt_kaixuan` | 当前双臂 URDF、STL 和独立模型查看器 |
| `src/kaixuan3_config` | MoveIt、控制器、运动学及启动配置 |
| `src/kaixuan_can_hardware` | IgH EtherCAT 硬件插件、O6 接口及直接控制工具 |
| `src/sts3215_driver` | STS3215 串口舵机驱动 |
| `src/normal` | 标定、运动、状态读取和测试脚本；不是 ROS 包 |
| `build`、`install` | 当前编译和安装空间；install 部分文件链接到 build 和 src |
| `diagnostics/linkerhand_o6_20260817` | O6 设备身份、PDO、XML 等接口参考 |
| `normal.txt` | 常用操作命令 |
| `readme.txt` | O6 操作说明 |
| `sts3215.txt` | STS3215 操作记录 |
| `KGU系列 V1.2使用说明(6).pdf` | 电机手册 |
| `kaixuan_urdf_front.png` | URDF 零位正视图 |
| `remote_rviz_client` | Ubuntu 22.04 / ROS 2 Humble 局域网 RViz 客户端与连接说明 |

## 编译及环境

```bash
cd /home/agxorin/cxl/kaixuan
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

现有安装使用软链接。单独删除 `build` 后不能继续使用旧 `install`，需要重新编译。
`log` 和 Python 的 `__pycache__` 是自动生成的，可在相关程序停止后清理。

## 自动运行诊断记录

原启动命令现在默认开启记录，无需另开终端：

```bash
source /home/agxorin/cxl/kaixuan/install/setup.bash
ros2 launch kaixuan3_config moveit_bringup.launch.py use_real_hardware:=true use_rviz:=false
```

每次启动独立保存到 ROS launch 日志目录中的 `flight_recorder/`：
通常是 `~/.ros/log/<启动时间>-<主机>-<PID>/flight_recorder/`，
使用 `ROS_LOG_DIR` 或其他用户启动时以终端显示的 `Flight recorder:` 路径为准。

| 文件 | 内容 |
| --- | --- |
| `joints_0000.csv` | 每 0.2 秒记录 14 个关节的实际角度、速度、控制器目标/反馈/误差、源时间戳及数据年龄；角度单位度，速度单位度/秒 |
| `controllers_0000.jsonl` | 约 5 Hz 保存控制器状态（控制器本身发布频率不变）；原始 ROS 单位 rad、rad/s |
| `hardware_0000.jsonl` | 约 5 Hz 的 PDO 快照：原始目标/编码器计数、速度、原始力矩、状态字/控制字、故障码；主站工作计数器、累计异常周期、最大循环间隔和延迟；另附位置差分速度供核对速度量纲 |
| `events_0000.jsonl` | ROS 日志、轨迹 action 状态、topic 轨迹指令、参数变化、机器人模型、启动参数和配置文件内容/哈希、记录器健康状态及正常退出标记 |
| `waveforms_0000.jsonl` | 异常触发时保存实际 EtherCAT 周期采样（默认 1 kHz）的压缩数据块，包含两臂异常前约 2 秒、后 10 秒；连续触发延长后窗口 |

### 瞬态故障证据（2026-09-15 更新）

常规三类采样文件统一约 5 Hz。每周期原始数据从实时线程经固定大小、无等待的队列传给诊断线程，
每约 50 ms 批量通过 `/kaixuan/cycle_capture` 发送给记录器。
正常时压缩波形只保留在内存中最近约 2 秒；故障/控制字/状态变化、跟随误差向上越过 1°、
主站状态变化、发送错误、丢帧或明显时序延迟会触发保存。两臂前窗口一起保存，后窗口持续 10 秒，
重复触发则延长。正常 Ctrl+C 时也保存剩余的两秒缓存。
启动阶段的正常使能状态转换也会产生事件和一个短波形窗口。

`drive_transition` 逐条保留真实源 `sample_steady_ns`、周期号、关节、变化前后的
错误码/状态字/控制字，以及原始位置/目标/速度/力矩和下发模式。
事件标志 1=CiA402/警告状态变化，2=控制字变化，4=错误码变化，8=跟随误差向上越过 1°。
不把目标到达位的普通抖动作为触发，但每周期的完整状态字仍存在波形中。
1°仅是日志触发阈值，不会停止、清错或改变电机控制。

波形包含每周期的计划/实际唤醒时间、迟到量、循环间隔、WKC、AL 状态、响应从站数、
链路状态，以及**前一周期** `ecrt_master_send` 返回值/耗时；并非实际从站 DC 相位测量。
错误瞬间的具体故障码通过逐周期事件保留，不再仅依赖 `last_nonzero_error_code`。
原始速度仍按设备反馈保存；`velocity_from_position_deg_s` 是 5 Hz 位置差分估计，
不能当成经过滤波的控制反馈，也没有更改原有速度换算。

启动配置阶段额外读取六个 uint16、subindex=0 对象：0x2033、0x2034、0x2035、
0x2063、0x2064、0x2065（与 `src/normal/pid.bash` 一致）。
`Drive parameter` 和设备身份通过 ROS 日志保存。只在该主站激活前读取，可能增加启动时间；
某从站读取失败则记录返回值/abort code 并跳过该从站剩余参数，不写入参数。
下述低频总线诊断会再次读取部分关键参数，可发现两次读取之间的参数变化。

异常波形每个 JSONL 行独立 zlib 压缩，可单独校验/解码。离线导出（不启动设备）：

```bash
source /home/agxorin/cxl/kaixuan/install/setup.bash
ros2 run kaixuan3_config cycle_capture.py /实际启动目录/flight_recorder/waveforms_*.jsonl --output /tmp/cycles.csv
```

CSV 内 master=0 表示右臂、master=1 表示左臂，axis1～axis7 为对应关节。
字段名及顺序也存于 `events` 的 `cycle_schema`。解码器拒绝覆盖已有输出文件。
`cycle_gap`、`cycle_ring_drops`、`cycle_decode_error` 和 recorder_health 的计数可识别采集不完整。
实时队列每主站最多缓存 2048 帧，消费者过慢时丢弃并计数，不会等待记录器。
采集启动不足两秒、队列/DDS/磁盘故障时，前窗口可能不完整。

每个文件达到 256 MiB 后生成下一编号，旧文件不自动删除。磁盘不足 1 GiB 时停止记录并在终端报错；运行前应检查空间，定期归档。
`recorder_health` 中的 `dropped_records`、`disk_error`、同步耗时用于判断记录是否完整。
JSONL 每行一条独立 JSON；异常终止后已完成的行仍可读，若最后一行残缺，分析时跳过该行。

记录器使用单独进程和写盘线程，每约 0.2 秒 flush + fsync，并同步新文件的目录。
这降低突然断电时的数据丢失，但不是断电零丢失保证；磁盘延迟、队列积压、存储设备缓存仍可能导致最后一段丢失。
若只切断电机电源而控制电脑仍运行，还可继续记录掉线后的反馈状态。
`recorder_stopped` 仅说明记录器正常退出；缺少该标记不能单独证明电机断电或进程崩溃。

`joint_state_age_s` / `controller_age_s` 大于 0.5 秒代表缓存数据已过时；没有数据的字段为空，未启用的手臂 `active_arm=0`，不能把缓存值当作实时反馈。
硬件诊断 `sample_steady_ns` 可用于判断 EtherCAT 循环是否仍在更新；累计 `fault_cycles` 和 `last_nonzero_error_code` 可保留发布间隔内的短暂故障线索。
`torque_raw` 是 0x6077 的原始值，**不是已标定的 N·m**。原始字段以十进制记录。
控制器参考目标与 PDO 实际下发的 `target_counts` 不同层级，应结合编码器计数、偏置/减速比一起分析。
常规快照不是完整 1 kHz 波形；异常波形按实际控制周期保存，以源周期号判断缺帧。
内存中的前窗口在整机突然断电时仍可能丢失，尤其故障触发批次尚未传输/落盘时；
没有后续供电就无法记录后 10 秒。不能保证仅凭日志定位所有故障。
记录器不会调用运动或使能服务；日志并非急停或故障保护功能。

发生问题后保留整个启动目录（含普通 ROS 日志和 `flight_recorder/`），提供大致发生时间和是否整机断电。
临时关闭记录可追加 `record_diagnostics:=false`。

### 通信与退出证据（2026-09-16 增补）

#### 驱动内部丢帧计数（2026-09-17 增补）

原 real-hardware launch 命令自动启用，结果写入 `flight_recorder/monitor64_*.jsonl`。
独立后台线程通过 EtherCAT CLI 执行：读原 `0x2002:00` → 临时写 int16 十进制 64 →
读回确认 → 读 `0x3000:00` 的 4 字节 int32 → 再确认选择 → 恢复原选择并确认。
不保存到 EEPROM，不写运动/复位/增益参数。运行时不要用其他工具同时修改监控选择项。

- 启动等待 5 秒；仅在最近 1 秒内收到该主站 OP/WKC 完整的周期数据时开始事务。
- 平时轮询全部 14 关节，事务结束后至少间隔 1 秒；right_joint1/2 距上次尝试超过 5 秒时优先补基线。
  优先采样不重置普通轮询游标，避免后面的关节长期得不到采样。实际间隔取决于邮箱响应和排队。
- 位置误差跨越 1°或非零驱动错误事件，会优先采对应关节，持续 30 秒；持续触发可延长。
  `counter_trigger` 保存源事件时间和最近基线；`counter_sample` 保存区间增量及是否观察到增长。
  这是异常触发，不是已证明的振荡识别，也不保证同步捕获瞬时丢帧。
- 记录主机时间、主站/从站、触发原因、原始字节、累计值及相邻采样增量。
  首次采样无增量；数值变小或出现负值标记 discontinuity，不猜测回绕/复位原因。
- 单条 CLI 命令限时 0.5 秒。读取失败且未写入或恢复已确认时，仅该关节退避 1/2/4/8/10 秒重试，
  其他关节继续采样；新异常可缩短退避至 1 秒以内（仍受全局事务间隔限制）。成功后清除失败累计。
  写入报错也尝试恢复；只有 `restore_failed` 等恢复未确认情况才停写并要求人工核查监控选择。
  `monitor64_retry`/停查会输出终端错误；健康日志每秒包含 worker 状态、各关节采样年龄及失败状态。
  停查、线程退出或样本超过 60 秒等不完整情况每 10 秒告警，不再静默丢失整场计数。
- 切换选择并读回后留出 100 ms 再读取，记录实际读取起止时间。此等待不是厂家确认的刷新保证；
  手册 Pn104 描述的是数码管刷新时间，不能直接当成 0x3000 的刷新周期。日志明确保留该不确定性。
- 正常退出尽力完成在途恢复；直接下电/SIGKILL 可能留下选择 64，不能保证恢复或最后落盘。
- 此线程不阻塞 PDO 实时线程，但增加邮箱通信负载（每事务最多 7 个命令），不包含在
  原 bus_probe 的每主站 10 请求/秒预算中。需结合周期迟到日志评估实际影响。
- `bus_diagnostics:=false` 会同时关闭本项和原 bus 探针；`record_diagnostics:=false` 也关闭本项。
  若无 EtherCAT 设备权限，记录失败并退避重试，不尝试 sudo 或交互密码。

下列旧表格和限速仅描述原 bus 探针，不包括这个独立采集线程。

原启动命令不变，默认 `bus_diagnostics:=true`。新增：

- `flight_recorder/bus_0000.jsonl`：每次异步只读查询的结果；与其他流相同，分段、约 0.2 秒同步落盘。
- `flight_recorder/events_*.jsonl`：增加 `esc_counter_change`、`drive_object_change`、
  `bus_probe_failure`、`bus_probe_gap`、`bus_probe_ring_drops` 和字段说明 `bus_probe_schema`。
- 启动目录根部 `hardware_shutdown.jsonl`：独立于 ROS 记录器的退出证据，由硬件进程在释放两条总线后写入并 fsync。

| 内容 | 对象 | 单关节名义最短读取间隔 |
| --- | --- | --- |
| 各端口无效帧、RX、转发错误、丢链路；ESC/PDI 处理错误计数 | ESC `0x0300` 起 20 字节 | 3 秒 |
| DC 滤波时钟差，保留原始值并按符号位＋幅值解释 | ESC `0x092C`，4 字节 | 3 秒 |
| 驱动温度原值 | CoE `0x2200:00` | 10 秒 |
| 丢帧容错阈值、监视选择项、PWM Off 延时、环路增益/滤波 | `0x2070/2002/2025/2031–2035/2063–2067:00` 中清单所列项 | 60 秒 |
| 可选同步事件遗漏、周期超限、同步错误 | `0x1C32/1C33:0B/0C/20` | 30 秒 |
| 可选同步模式及周期 | `0x1C32/1C33:01/02` | 60 秒 |
| 可选软件版本字符串 | `0x100A:00` | 300 秒 |

KGU 对象依据工作区《KGU系列 V1.2使用说明(6).pdf》印刷页 15–17、25、30、56–58。
上述 bus 探针不写参数；2026-09-17 新增的独立 monitor64 采集会临时写 `Pn002` 并恢复，见下节。
不写 `Pn070`、增益、滤波或错误清零；原有自动复位和控制逻辑不变。
标准 ESC 字段依据 [Beckhoff ESC 寄存器说明](https://download.beckhoff.com/download/document/io/ethercat-development-products/ethercat_esc_datasheet_sec2_registers_3i0.pdf)，
可选 CoE 同步字段参见 [同步对象说明](https://infosys.beckhoff.com/content/1033/ethercatsystem/2469122443.html)。
KGU 手册没有承诺这些可选 CoE 对象存在：失败只表示本次未读到，不能当成数值零或直接认定“不支持”。

每主站最多每秒发起 10 个请求，最多一个请求在途；每 10 ms 检查一次异步状态，
总线 OP/WKC 完整后至少等待 5 秒才开始查询，总线不就绪时不发起新请求。
查询轮转覆盖 7 个关节，首次完整参数扫描需要几十秒；上表是间隔下限，
排队、启动和超时会延长实际间隔，必须看每项的请求/完成时间，不能把它们当作同步采样。
`completion_steady_ns` 是主机观察到请求完成的时间，不是驱动内部精确采样时间。
实时线程不写盘、不运行阻塞式 `sdo_upload`，只调度 IgH 的异步读请求。
`max_bus_probe_work_ns`、原有循环迟到/间隔和发送耗时用于检查诊断自身的影响；只读查询也有通信和 CPU 开销。

可选对象首次失败后本场不再查询；手册明确的对象失败后至少等待 60 秒，连续三次失败则本场停查该项。
SDO 超时设置为 300 ms；若请求仍 busy 超过 1 秒，记录一次超时并暂停该主站后续探测，
绝不覆盖在途请求。PDO 控制循环继续原有行为。失败值为 `null`，不会拿上次成功值冒充新值。
原始字节、返回状态、停查标记和读数间隔全部保存；此 IgH 异步 API 不提供 CoE abort code，需结合节点/内核日志。
`/kaixuan/bus_diagnostics` 每秒空心跳只代表发布线程存活，不代表某对象读取成功；查看 `bus` 中的完成时间及失败记录。

**仍有明确盲区：**

- 手册 `Pn002=63/64` 是两种 EtherCAT 丢帧计数的“监视选择项”，不是 CoE 地址或子索引。
  厂家现已提供 `Pn002=64` 后读取 `0x3000:00` 的方法，设备字典确认 int32，见下节。
  仍需厂家说明次数 1/2 的具体触发条件和 Er400 判定规则。
- ESC 计数不等于驱动 Er400 内部计数；启动时的非零值不能直接归因于本次抖动。
  保存相邻成功读数的增减，减小仅标记为复位/回绕待核对；8 位计数可能饱和，未实现的 ESC 字段也可能读为零。
- DC `0x092C` 是滤波时钟差，不是“PDO 到达距离 SYNC0 还有多少时间”；低频读取会漏掉短瞬态。
  目前不能直接看到驱动内部的数据消费时刻、抱闸实际状态和未公开的监视值。

退出文件记录关闭请求、循环结束、线程 join、主站 deactivate/release 的时间，
以及停止前快照、最后 PDO 内存中的实际/目标/状态字/控制字/错误码/力矩。
`final_pdo_available=0` 时忽略对应数值；即便可用，它也只是最后缓存，**不能证明主站释放后电机的状态**。
没有为了日志追加停机命令或等待反馈。若只有电机掉电而电脑仍运行，常规记录可以继续；
整机断电、SIGKILL 或崩溃可能没有退出文件，正常退出写入也不保证突然断电时零丢失。

仅关闭新增总线查询：`bus_diagnostics:=false`；原角度/故障波形仍保留。
`record_diagnostics:=false` 只关闭 Python 记录器；若要同时关闭新增查询，两个参数都设 false。
独立退出文件仍保留，便于检查硬件关闭过程。

离线验证（不会启动设备）：

```bash
cd /home/agxorin/cxl/kaixuan
source /opt/ros/humble/setup.bash
ctest --test-dir build/kaixuan_can_hardware --output-on-failure
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 python3 -m unittest discover -s src/kaixuan3_config/test -v
```

## 历史清理

移除了历史日志、Python 缓存、`src/normal` 中误生成的构建目录、
旧 ROS 1 启动文件、导出日志、无关代理说明和空元数据目录。
保留全部当前 ROS 包、模型资源、维护及测试脚本。

删除项归档：
`/home/agxorin/cxl/kaixuan_cleanup_backups/20260914T074433Z/removed.tar.gz`
同目录的 `manifest.json` 列出清理项。需要恢复时建议先解压到单独目录再选取文件。
