# MoveIt + ros2_control 双臂集成说明

本 workspace 现在按下面的结构工作：

- `master0` 对应右臂 7 轴
- `master1` 对应左臂 7 轴
- 底层协议是 `NIIC-EtherCAT-Demo-4.1.4-NECRO` 里的 CiA402 CSP
- MoveIt 规划组包括 `right_arm`、`left_arm`、`dual_arms`

计数换算按每条手臂内部轴序固定为：

- `joint1 ~ joint4`：一圈 `524288`
- `joint5 ~ joint7`：一圈 `262144`

## 包结构

- `kaixuan1`：双臂 URDF/mesh
- `kaixuan3_config`：MoveIt 配置、双臂控制器、launch
- `kaixuan_can_hardware`：基于 NIIC EtherCAT 的 `ros2_control` 硬件插件

## 1. 仿真模式

只启动双臂 mock ros2_control：

```bash
cd /home/niic/cxl/ros2_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaixuan3_config control_only.launch.py
```

启动完整双臂 MoveIt 仿真：

```bash
cd /home/niic/cxl/ros2_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaixuan3_config moveit_bringup.launch.py use_real_hardware:=false
```

## 2. 真机模式

推荐直接用统一 launch：

```bash
cd /home/niic/cxl/ros2_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaixuan3_config moveit_bringup.launch.py \
  use_real_hardware:=true \
  use_rviz:=true
```

如果当前用户没有实时调度权限，`ros2_control_node` 可能会在启动 EtherCAT 主站前直接报 `Operation not permitted`。这套硬件插件默认会申请 `priority:=90`，因此在未配置 `rtprio` / capability 的机器上，真机 bringup 需要在 root shell 内重新 source ROS 环境后再启动：

```bash
cd /home/niic/cxl/ros2_control
sudo -E bash -lc 'source /opt/ros/humble/setup.bash && source /home/niic/cxl/ros2_control/install/setup.bash && ros2 launch kaixuan3_config moveit_bringup.launch.py use_real_hardware:=true use_rviz:=false'
```

需要图形界面时，建议等真机控制链路起来后，再用普通用户单独启动 RViz-only：

```bash
cd /home/niic/cxl/ros2_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaixuan3_config rviz_only.launch.py use_real_hardware:=true
```

这个 launch 会：

- 先执行 `xmlParser`
- 启动 EtherCAT 双主站硬件插件
- 拉起 `joint_state_broadcaster`
- 拉起 `right_arm_controller`
- 拉起 `left_arm_controller`
- 启动 `move_group`
- 启动 RViz

如果你只想启动硬件和控制器，不启 MoveIt：

```bash
cd /home/niic/cxl/ros2_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaixuan_can_hardware bringup.launch.py
```

常用真实硬件参数：

```bash
ros2 launch kaixuan3_config moveit_bringup.launch.py \
  use_real_hardware:=true \
  use_rviz:=false \
  cycle_time_ns:=1000000 \
  master0_affinity:=0 \
  master1_affinity:=1 \
  comp0:=0.0 \
  comp1:=0.0 \
  comp2:=0.0 \
  comp3:=0.0 \
  comp4:=0.0 \
  comp5:=0.0 \
  comp6:=0.0 \
  left_comp0:=0.0 \
  left_comp1:=0.0 \
  left_comp2:=0.0 \
  left_comp3:=0.0 \
  left_comp4:=0.0 \
  left_comp5:=0.0 \
  left_comp6:=0.0
```

说明：

- `comp0 ~ comp6` 是右臂 7 轴补偿角度，单位度
- `left_comp0 ~ left_comp6` 是左臂 7 轴补偿角度，单位度
- 不传 `eni_file` 时默认按 ESI 模式工作

## 3. 远端 RViz-only 控制双臂

保留你之前的使用方式。

机器人本机运行：

```bash
cd /home/niic/cxl/ros2_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaixuan3_config moveit_bringup.launch.py \
  use_real_hardware:=true \
  use_rviz:=false
```

另一台机器加入同一个 ROS 2 网络后，只启动 RViz：

```bash
cd /home/niic/cxl/ros2_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kaixuan3_config rviz_only.launch.py use_real_hardware:=true
```

这个 `rviz_only.launch.py`：

- 不启动 EtherCAT
- 不启动控制器
- 不启动 `move_group`
- 只启动 RViz，并加载双臂 robot_description / SRDF / planning 配置

因此另一台机子仍然可以通过 RViz 控制左右机械臂。

## 4. MoveIt 执行链路

MoveIt 通过标准 ros2_control 链路执行：

1. `joint_state_broadcaster` 发布 14 轴 `/joint_states`
2. `move_group` 将规划结果发到左右 `FollowJointTrajectory`
3. `JointTrajectoryController` 调用 EtherCAT 硬件插件
4. 硬件插件把命令写到 CiA402 `0x607A`，并维持 `0x6060 = 8`、`0x6040 = 6 -> 7 -> 15`

## 5. 关键接口

- Topic: `/joint_states`
- Action: `/right_arm_controller/follow_joint_trajectory`
- Action: `/left_arm_controller/follow_joint_trajectory`
- Controller: `joint_state_broadcaster`
- Controller: `right_arm_controller`
- Controller: `left_arm_controller`
- Planning groups: `right_arm` / `left_arm` / `dual_arms`

## 6. 编译

```bash
cd /home/niic/cxl/ros2_control
source /opt/ros/humble/setup.bash
colcon build --packages-select tilt_kaixuan kaixuan3_config kaixuan_can_hardware
source install/setup.bash
```

如果构建时报找不到 `hardware_interfaceConfig.cmake`，说明当前机器没有装完整的 `ros2_control` 开发包，需要先补齐 ROS 2 控制相关系统包。

## 7. 成功标志

启动成功后，至少应看到：

```bash
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
ros2 topic echo /joint_states
```

期望结果：

- `joint_state_broadcaster` 已激活
- `right_arm_controller` 已激活
- `left_arm_controller` 已激活
- 左右两个 `follow_joint_trajectory` action 都存在
- `/joint_states` 持续输出 14 个关节状态
