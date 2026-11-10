# 左臂规划示例：每种算法一个 Python 文件

**直接修改各个 `.py` 文件顶部的参数，再运行这个文件。** 每个文件只运行一种算法，默认直接执行并返回起点。
固定使用左臂 `left_arm` / `left_joint1..7`，末端 `left_link7`，坐标系 `base_link`。

## 1. 选择对应文件

| 算法 | 文件 | 文件顶部的专用参数 |
| --- | --- | --- |
| RRT* | [ompl_rrtstar.py](ompl_rrtstar.py) | `PLANNER_PARAMETERS`：`range`、`goal_bias`、`delay_collision_checking` |
| RRTConnect | [ompl_rrtconnect.py](ompl_rrtconnect.py) | `PLANNER_PARAMETERS`：`range` |
| PRM | [ompl_prm.py](ompl_prm.py) | `PLANNER_PARAMETERS`：`max_nearest_neighbors` |
| EST | [ompl_est.py](ompl_est.py) | `PLANNER_PARAMETERS`：`range`、`goal_bias` |
| Pilz CIRC | [pilz_circ.py](pilz_circ.py) | 圆弧终点、`VIA_POSITION_M` 或 `VIA_OFFSET_M` |
| Cartesian Path | [cartesian_path.py](cartesian_path.py) | `WAYPOINT_OFFSETS_M` / `WAYPOINT_POSITIONS_M`、`MAX_STEP_M`、跳变阈值 |
| Pilz LIN | [pilz_lin.py](pilz_lin.py) | 直线终点位置、姿态、速度和加速度 |

旧的合并入口和 `demo_config.yaml` 已删除。`planning_common.py` 提供通信、规划请求、执行和结果保存，
示例的目标、运动设置和算法调参值由当前 `.py` 文件传入。

## 2. 各种输入怎么改

先读取当前左臂关节角度（只读反馈，不发送运动指令）：

```bash
python3 read_left_joints.py
python3 read_left_joints.py --watch  # 持续刷新，Ctrl+C 退出
```

终端须先加载 ROS 环境，并使用与 bringup 相同的 `ROS_DOMAIN_ID`。
脚本按 `left_joint1..7` 顺序打印弧度、角度，以及可复制的 `TARGET_JOINTS_DEG` 一行。
当前反馈表示起始姿态；规划目标按你希望到达的关节角度填写。

**四个 OMPL 文件统一只使用 `TARGET_JOINTS_DEG`**：按 `left_joint1..7` 的顺序填写七个绝对关节角，单位度。
四个文件初始为 `None`，运行前须填入你的实际目标；未填写会在连接 ROS 前报错。
填写同一组角度可比较四种算法，格式示例：

```python
TARGET_JOINTS_DEG = [0, 0, 0, 10, 0, 0, 0]  # 七个绝对关节角，单位度
```

每个 OMPL 文件各有自己的 `PLANNER_PARAMETERS`，只修改当前文件。例如在 `ompl_rrtstar.py`：

```python
PLANNER_PARAMETERS = {
    "range": 0.15,
    "goal_bias": 0.1,
    "delay_collision_checking": 1,
}
```

`range` 是七轴配置空间的扩展距离，0 表示自动选择；七轴目标角度修改 `TARGET_JOINTS_DEG`。
RRT* 的优化时间可修改 `PLANNING_TIME_S`。参数会写入对应左臂算法配置，并通过 MoveIt 读回确认。

**Pilz LIN**：在 `pilz_lin.py` 中修改 `TARGET_OFFSET_M`，或绝对 `TARGET_POSITION_M`，
以及 `TARGET_RPY_DEG`。LIN 使用末端位姿终点和运动缩放。

**Pilz CIRC**：在 `pilz_circ.py` 中修改圆弧终点和经过点：

```python
TARGET_OFFSET_M = [0.0, 0.0, 0.02]
VIA_OFFSET_M = [0.0, 0.002, 0.0]  # 圆弧中间点 = 起终点中点 + 此偏移，单位米
VIA_POSITION_M = None              # 或直接填绝对经过点 [x, y, z]，优先于 VIA_OFFSET
```

起点为当前末端，起点、经过点、终点不可共线。终点姿态可用 `TARGET_RPY_DEG` 指定。

**Cartesian Path**：在 `cartesian_path.py` 中直接填写按顺序经过的点：

```python
WAYPOINT_OFFSETS_M = [
    [0.0, 0.0, 0.02],
    # [0.005, 0.0, 0.02],  # 可增加后续途经点
]
WAYPOINT_POSITIONS_M = None  # 或填绝对 [[x1,y1,z1], [x2,y2,z2], ...]
MAX_STEP_M = 0.001           # 插值步长，米
```

每个相对点均相对于本次起始末端位置，**不是相对于前一个点**。
姿态由 `ORIENTATION_RPY_DEG` 设置，`None` 表示保持起始姿态。开启碰撞检查，要求完成比例为 1。

## 3. 每个文件自己的运行参数

七个文件的顶部各有一份，可分别修改：

```python
VELOCITY_SCALE = 0.1
ACCELERATION_SCALE = 0.01
PLANNING_TIME_S = 3.0
REPEAT = 1
PAUSE_S = 2.0
EXECUTE = True
RETURN_TO_START = True
```

默认执行后用 RRTConnect 返回本次运行前的左臂关节位置。
想停在目标处，将当前文件的 `RETURN_TO_START = False`；重复执行时保持 True。
`RETURN_RRTCONNECT_RANGE` 仅控制返回起点的规划；RRTConnect 示例的返回直接使用本文件的 `range`。

左臂关节加速度基准仍为 **2.0 rad/s²**，实际配置在
`src/kaixuan3_config/config/joint_limits.yaml`，URDF 中有对应说明。
`ACCELERATION_SCALE` 是缩放倍率。默认 0.01 沿用已验证的模拟零位参数。
OMPL 的缩放作用于关节限制，Pilz LIN/CIRC 的缩放作用于笛卡尔限制，之后还需满足关节限制。
因此不同规划方式使用相同缩放时，末端速度也可能不同。

## 4. SSH 中运行

如果已经启动当前工作区的左臂 MoveIt，直接运行示例。否则在终端一启动真实硬件：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=26  # 两个终端必须一致；此处沿用当前 bringup 的 domain
ros2 launch kaixuan3_config moveit_bringup.launch.py \
  use_real_hardware:=true active_arm:=left use_rviz:=false
```

模拟控制器使用 `use_real_hardware:=false`。真实驱动启动会使能保持当前位置。
等 `You can start planning now!`，网格首次加载约需一分钟。
Python 参数修改保存后直接重跑，不需要编译或重启 MoveIt。

终端二加载环境，选择一条命令运行：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=26
cd src/normal/path_planing

python3 ompl_rrtstar.py
python3 ompl_rrtconnect.py
python3 ompl_prm.py
python3 ompl_est.py
python3 pilz_circ.py
python3 cartesian_path.py
python3 pilz_lin.py
```

上述每条命令均执行一种算法。Ctrl+C 请求取消左臂控制器当前目标，不会自动返回起点。
真实 EtherCAT 硬件在规划执行前检查 `/kaixuan/hardware_diagnostics`：DC 资格、总线状态和七轴使能。
执行中持续监视诊断，并每秒输出实际位置变化和跟踪误差；硬件失锁时取消当前目标并报出原因。
模拟控制器自动跳过 EtherCAT 专用诊断。

脚本启动时会打印实际 `ROS_DOMAIN_ID` 和 RMW。未设置 domain 时 ROS 默认使用 0，
无法连接当前 domain 26 中的 bringup。控制器硬件服务发现最多等待 15 秒；
若提示 `/controller_manager/list_hardware_components` 不可用，先核对两个终端的 ROS 环境和 domain，
再确认 bringup 是否运行。这个连接错误本身不代表 EtherCAT 故障。

如果能读到角度、控制器也接受轨迹，但机械臂不动，查看硬件诊断。
本机曾出现 `dc_follower_failed=1`、`dc_follower_error=7`（`QualityLost`），
硬件保护将控制字保持为 6、驱动未使能；此时读取位置和规划仍可正常工作。
该失锁会锁存，需要退出并重新启动 bringup 重新完成时钟资格；若再次发生，应排查 DC 跟随问题。

仅查看某个文件的最终参数，或只规划而不执行：

```bash
python3 ompl_rrtstar.py --show-config
python3 ompl_rrtstar.py --plan-only
```

结果自动保存在 `results/<时间>/`：`request.json` 记录当前参数文件和实际输入，
`summary.json/csv` 保存结果、耗时、轨迹时长与路径长度，轨迹 CSV 包含七轴位置/速度/加速度。
可用 `--output-dir /tmp/my-test` 修改输出位置。

## 5. 添加必须绕开的虚拟障碍物

在 [scene_obstacle.py](scene_obstacle.py) 顶部修改球体中心和半径，位置相对于 `base_link`，单位米：

```python
OBSTACLE_CENTER_M = [0.258365, 0.347649, -0.628524]
OBSTACLE_RADIUS_M = 0.06
```

此位置用于从零位附近到 `[-20, 20, 30, 50, -10, -20, 0]` 度的对比：
起点、终点均不碰撞，而原关节直线 51 个采样姿态中有 19 个与球体碰撞。
球体作为 `CollisionObject` 参与 MoveIt 的机器人碰撞检查，规划必须绕开它。

在本目录、加载 ROS 环境后运行：

```bash
export ROS_DOMAIN_ID=26
python3 scene_obstacle.py add
python3 scene_obstacle.py status
python3 ompl_rrtconnect.py  # 或其他 OMPL 文件
python3 scene_obstacle.py remove
```

`scene_obstacle.py` 只添加、查询或移除指定球体，不发送机械臂运动指令。
修改参数后再次运行 `add` 才会更新场景；`remove` 只删除 `left_arm_demo_obstacle`，保留其他障碍物。
四个算法继续使用各自 Python 文件中的参数。比较时保持接近的起点；
如果已在目标角度，重跑相同目标不会经过中间障碍区域。
自动返回仍由各文件的 `RETURN_TO_START` 决定。

本次绕障测试记录在 [results/ompl-obstacle-20261105-075118/comparison.md](results/ompl-obstacle-20261105-075118/comparison.md)。
测试结束会移除本次球体，原先无障碍场景下的行为随之恢复。

## 6. PRM 路网预建、保存和后续查询

[prm_roadmap.py](prm_roadmap.py) 使用原生 OMPL PRM 建图，保存到文件后供 MoveIt 加载。
**该工具始终只规划，不执行机械臂。** 查询终点和运动缩放从 `ompl_prm.py` 读取，
预建时间在 `prm_roadmap.py` 顶部的 `BUILD_TIME_S` 修改，默认 10 秒。

```bash
export ROS_DOMAIN_ID=26
python3 scene_obstacle.py add  # 如需复现带球体的场景
python3 prm_roadmap.py build
python3 prm_roadmap.py query
python3 prm_roadmap.py query   # 再次加载已保存的路网，不重新预建
python3 scene_obstacle.py remove
```

首次 `build` 会用 CMake 编译 `prm_roadmap_native/` 中的小型建图程序，不需要重启 bringup。
它从当前 MoveIt 获取 URDF、SRDF、碰撞场景和状态，使用相同的关节状态空间及碰撞检查持续扩展路网，
随后保存 `roadmap.graph`。建图进程没有运动接口。
`graph.json` 保存节点七轴角度、连接边和边代价，以及一条验证路径的累计 `g`、剩余估计 `h` 和 `f=g+h`。
`g/h` 属于本次查询，不是路网节点永久不变的属性。

缓存位于 `results/prm_roadmaps/<场景指纹>/`。指纹包含机器人模型、障碍几何、碰撞允许矩阵、
附着物和其他机器人关节状态，排除左臂自身位置与时间戳，因此左臂换起点仍可查询同一张图。
障碍物、模型或其他固定条件改变后，会选择另一缓存并要求重新建图。
保存的是有限采样路网，并不保证覆盖全部七维自由空间。

查询使用已有 MoveIt 的原生 PRM 和路径后处理；搜索耗时、服务总耗时、轨迹时长分别输出。
预建成本只在建图时支付，文件加载、连接新起终点、A* 和路径简化仍需要时间。
普通 `ompl_prm.py` 保留原来的单次规划示例；加载预建路网使用本节的 `query` 命令。

## 7. 检查与原有记录

2026-11-05 已在当前真实左臂上运行 `ompl_rrtstar.py`：关节 1 从约 -5.13° 到 19.94°，
随后返回约 -5.16°，去程和返回均成功。观测期间 DC 保持锁定、WKC 为 21；
运行记录在 [results/diagnosis-service-discovery-20261105/real_rrtstar/summary.json](results/diagnosis-service-discovery-20261105/real_rrtstar/summary.json)，
实际角度与总线观测在 [real_motion_observation.json](results/diagnosis-service-discovery-20261105/real_motion_observation.json)。
本次仅调整 Python 服务发现等待和环境提示，未修改 EtherCAT 实现或参数。
26 项离线检查通过；更新后的真机连接检查通过，错误 domain 的报错提示也已实际验证。

此前七个独立入口均已在模拟 domain 119 中规划成功，LIN 的默认执行并返回起点验证通过，
13 项离线检查通过。记录见
[results/validation-independent-python-20261105/validation.json](results/validation-independent-python-20261105/validation.json)。

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s src/normal/path_planing -p 'test_*.py' -v
```

此前七种算法的模拟执行与取消记录保存在 `results/validation-20261105/`。
停止 bringup 时仍有[工作区原有说明](../../../README.md)中的 `move_group` 析构问题。
