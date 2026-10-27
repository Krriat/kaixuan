# Kaixuan 局域网 RViz 客户端

适用：你的电脑是 Ubuntu 22.04，已安装 ROS 2 Humble，与机器人处于同一局域网。
本包为模型、配置和 Python / Bash 脚本，不含 ARM 二进制，也不需要 colcon 编译。

电脑端只运行 RViz；机器人端运行现有的 MoveIt `move_group`、状态发布器和 ros2_control。
RViz 订阅 `/joint_states`、`/tf`、`/tf_static`、`/monitored_planning_scene`，
通过 `/move_action` 和 `/execute_trajectory` 等 ROS 2 action / service 请求规划及执行。
因此机器人端必须保持 bringup 运行，只有话题订阅不能完成运动控制。

## 1. 从机器人复制到自己的电脑

在你自己的 Ubuntu 电脑终端运行：

```bash
scp agxorin@172.16.13.197:/home/agxorin/cxl/kaixuan/kaixuan_remote_rviz_humble.tar.gz ~/
cd ~
tar -xzf kaixuan_remote_rviz_humble.tar.gz
cd remote_rviz_client
```

如果 SSH 使用其他用户名、IP 或端口，替换 scp 中对应项。
模型网格在 `model/meshes/`；保留整个解压目录，路径中含空格也可以启动。

## 2. 机器人端：启动左臂

在机器人 SSH 终端运行：

```bash
source /home/agxorin/cxl/kaixuan/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch kaixuan3_config moveit_bringup.launch.py \
  use_real_hardware:=true \
  use_rviz:=false \
  active_arm:=left
```

也可以在机器人上的客户端目录运行 `bash ./robot_bringup.sh`，效果相同，另会统一 DDS UDP 配置。
该脚本只在机器人上使用；电脑端使用下一步的 `start_rviz.sh`。
切换网络环境变量时，应先结束原 bringup，再在同一配置下启动；避免重复启动硬件控制节点。

## 3. 自己的电脑：检查连接并打开 RViz

```bash
cd ~/remote_rviz_client
bash ./check_connection.sh
bash ./start_rviz.sh
```

如果提示缺少 RViz / MoveIt 依赖，先运行一次：

```bash
bash ./install_dependencies.sh
```

脚本自动 source `/opt/ros/humble/setup.bash`，设置 `ROS_DOMAIN_ID=0`、
`ROS_LOCALHOST_ONLY=0`、Fast DDS，并加载本地模型、SRDF、运动学和关节限制。
启动的唯一 ROS 节点是本机 RViz，不启动本地 `move_group`、模拟关节发布器或控制器。
连接检查脚本只读状态与查询服务，不发送运动目标。

RViz 默认选择 `left_arm`：

1. 确认模型姿态跟随机器人反馈，Displays 中的 MotionPlanning 已连接。
2. 在 MotionPlanning 面板将 Start State 设为 `<current>`，Planning Group 选择 `left_arm`。
3. 拖动末端交互标记设置目标；也可在 Joints 页设置关节目标。
4. 将速度、加速度比例先设为 `0.1`，点击 **Plan** 查看轨迹。
5. 确认轨迹后点击 **Execute**。此操作会让真实左臂运动。

RViz 使用 ROS 关节坐标，驱动的硬件标定偏置仍在机器人端处理。
右臂未启用时不选择 `right_arm` / `dual_arms` 作为执行组。
如果机器人启动了其他手臂，可使用：

```bash
bash ./check_connection.sh --arm right
bash ./start_rviz.sh arm:=right
# 双臂：两条命令分别使用 --arm both 和 arm:=both
```

## 4. 网络排查

两端的 `ROS_DOMAIN_ID` 必须相同。修改 `network.env` 中的 `KAIXUAN_ROS_DOMAIN_ID` 后，
机器人端也要使用相同值。当前机器人地址为 `172.16.13.197`，地址变化时更新复制命令。
ROS 2 通过 DDS 在局域网发现节点，不使用 ROS 1 的 `ROS_MASTER_URI`。

如果 SSH 能连接但检查失败：

- 确认两台电脑在同一网络，路由器没有开启无线客户端隔离，VPN 没有改变路由。
- 确认两端不是 `ROS_LOCALHOST_ONLY=1`，机器人仍在运行 `move_group` 和左臂控制器。
- 如果启用防火墙，需要允许两台主机之间的双向 UDP 流量；仅放行 SSH 22 端口不足以承载 DDS。
- 运行 `timedatectl status` 检查时钟同步；本机与机器人的时间差较大会影响状态新鲜度判断。
- 原先在其他 Domain 下启动过 ROS CLI daemon 时，在新环境中执行 `ros2 daemon stop` 再检查。
  包内 `check_connection.sh` 不依赖 CLI daemon。

### Wi-Fi 不转发组播：可选 Discovery Server

仅在普通自动发现失败时使用。保留机器人上的这个终端：

```bash
cd /home/agxorin/cxl/kaixuan/remote_rviz_client
bash ./discovery_server.sh
```

机器人另开一个 SSH 终端，用统一配置启动：

```bash
cd /home/agxorin/cxl/kaixuan/remote_rviz_client
KAIXUAN_DISCOVERY_SERVER=172.16.13.197:11811 bash ./robot_bringup.sh
```

自己的电脑上：

```bash
cd ~/remote_rviz_client
export KAIXUAN_DISCOVERY_SERVER=172.16.13.197:11811
bash ./check_connection.sh
bash ./start_rviz.sh
```

两端都必须使用同一 Discovery Server 设置，并保持 server 运行。该方案替代组播发现，
状态、规划和执行数据仍直接在两台电脑间传输，仍需要双向 UDP 可达。
返回普通发现时取消 `KAIXUAN_DISCOVERY_SERVER` 设置，并重启两端 ROS 节点。

## 文件与模型版本

- `start_rviz.sh`：电脑端 RViz 入口；`arm:=left` 为默认值。
- `check_connection.sh`：只读检查反馈、TF、规划/执行接口、控制器及 URDF 关节结构。
- `robot_bringup.sh`：机器人端已有工作区的启动入口；不会安装或修改驱动。
- `model/`：当前 URDF 与全部 15 个 STL 网格。
- `config/`：当前 SRDF、运动学、关节限制、RViz 配置、DDS UDP 配置。
- `MANIFEST.sha256`：文件校验清单，可运行 `sha256sum -c MANIFEST.sha256`。

机器人 URDF / 网格 / SRDF 改动后，应重新复制匹配的客户端文件。所有操作仍经过机器人端 MoveIt。

参考：[MoveIt Humble RViz / MoveGroup 接口](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html)、
[ROS 2 Humble 环境变量](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)、
[Fast DDS 2.6 Discovery Server](https://fast-dds.docs.eprosima.com/en/v2.6.10/fastdds/discovery/discovery_server.html)。
