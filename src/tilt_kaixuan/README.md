# 整体 URDF 模型使用说明

本工程用于在 ROS 2 Humble 的 RViz2 中显示整体机器人模型，并通过可手动输入角度的界面调整 14 个关节。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- RViz2
- Python 3 + PyQt5
- `robot_state_publisher`

## 主要文件

```text
tilt_kaixuan/
├── urdf/整体urdf.urdf       # 机器人结构、关节和网格引用
├── meshes/                    # STL 网格文件
├── config/rviz2.rviz          # RViz2 显示配置
├── scripts/joint_angle_gui.py # 关节角度手动输入面板
└── launch/display_ros2.sh      # ROS 2 一键启动脚本
```

## 修改 URDF 后如何打开

### 1. 进入工程目录

```bash
cd /home/agxorin/cxl/kaixuan/src/tilt_kaixuan
```

### 2. 检查 URDF 语法和连杆树

```bash
check_urdf "urdf/整体urdf.urdf"
```

正常时会看到：

```text
Successfully Parsed XML
root Link: world
```

如果出现 XML 报错，先根据报错行号修正 URDF，不要继续启动 RViz。

### 3. 关闭旧的 RViz

修改 URDF 后不会自动热加载。先关闭已打开的 RViz 窗口，或在启动它的终端中按：

```text
Ctrl+C
```

### 4. 重新启动

首次运行时可先添加执行权限：

```bash
chmod +x launch/display_ros2.sh
```

然后启动：

```bash
./launch/display_ros2.sh
```

脚本会自动打开：

- `robot_state_publisher`
- RViz2
- 关节角度手动输入面板

关闭 RViz 后，脚本会结束其他相关进程。

## 修改 URDF 后是否需要编译

使用独立模型查看器时不需要编译。

`display_ros2.sh` 会直接读取工程中的：

```text
urdf/整体urdf.urdf
```

因此修改并保存 URDF 后，只需关闭并重新运行启动脚本。

当前包名为 `tilt_kaixuan`，已使用 ROS 2 `ament_cmake` 构建系统。
旧 ROS 1 的 display.launch 和 gazebo.launch 已清理。
在 MoveIt 工作空间中使用时，从工作空间根目录编译：

```bash
cd /home/agxorin/cxl/kaixuan
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tilt_kaixuan
source install/setup.bash
```

## 关节类型和限位

左右机械臂共 14 个可动关节，已全部从 `continuous` 改为标准 `revolute` 关节。`world_to_base` 仍为 `fixed` 关节。

每个机械臂关节统一使用：

```xml
<limit
  lower="-3.141592653589793"
  upper="3.141592653589793"
  effort="100"
  velocity="3.141592653589793" />
```

参数含义：

- 角度下限：`-180°`
- 角度上限：`+180°`
- 最大 effort：`100`
- 最大速度：`π rad/s`（`180°/s`）

关节的实际动态旋转方向由各自的 `<axis xyz="..." />` 定义；`<origin rpy="..." />` 定义关节零位坐标系的安装方向，不是实时关节角。

## 手动输入关节角

在“URDF 关节角度输入”窗口中：

1. 点击关节右侧的数值框。
2. 按 `Ctrl+A` 选中原数值。
3. 输入角度，例如 `-10`。
4. 按 `Enter` 生效。

界面单位为度，支持负数和小数。所有关节的输入范围都是 `-180°～+180°`。

部分关节已做零位标定。界面角度与 `/joint_states` 底层原始角度的关系为：

```text
底层原始角度 = 界面显示角度 + 零位偏置
```

当前默认值和标定值：

| 关节 | 零位偏置 | 界面默认角度 | 底层默认角度 |
|---|---:|---:|---:|
| `left_joint2` | `+180°` | `-10°` | `170°` |
| `right_joint1` | `+180°` | `0°` | `180°` |
| `right_joint2` | `-160°` | `-10°` | `-170°` |
| `right_joint3` | `+180°` | `0°` | `180°` |
| 其他关节 | `0°` | `0°` | `0°` |

因此，不能只看 URDF 中的 `origin rpy` 判断当前姿态；`robot_state_publisher` 还会在该固定姿态之后应用 `/joint_states` 中的实时关节角。

点击“全部置零”时，界面上的标定角度会全部变为 `0°`；已标定关节的底层角度仍会包含上表中的零位偏置。

默认角度和零位偏置在以下文件中配置：

```text
scripts/joint_angle_gui.py
```

## base_link 高度

URDF 中已添加 `world` 根坐标系，默认变换为：

```xml
<joint name="world_to_base" type="fixed">
  <origin xyz="0 0 1" rpy="0 0 0" />
  <parent link="world" />
  <child link="base_link" />
</joint>
```

这表示 `base_link` 相对 `world` 的 Z 轴高度为 `1 m`。要修改高度，修改 `xyz` 的第三个数值，然后重启查看器。

## ROS 域隔离

启动脚本默认使用：

```bash
ROS_DOMAIN_ID=77
```

这可以防止其他 ROS 工程的 `/robot_description` 覆盖当前模型。

如果要在另一个终端中查看当前节点、话题或 TF，先执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
```

然后可以使用：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo world base_link
```

## 常见问题

### RViz 报 `Package does not exist`

应使用 `./launch/display_ros2.sh` 启动，不要直接双击 URDF。脚本会把 `package://tilt_kaixuan/...` 网格路径解析为当前工程的绝对路径。

### RViz 显示了其他机器人

确保使用本工程的启动脚本。默认 ROS 域 77 已与其他工程隔离。

### 修改 URDF 后没有变化

RViz 不会自动重载 URDF。关闭当前 RViz，然后重新运行：

```bash
./launch/display_ros2.sh
```

### URDF 的 `rpy` 没变，模型却旋转了

`joint/origin rpy` 只是关节零位坐标系的固定安装姿态。完整姿态还包含角度面板发布到 `/joint_states` 的实时关节角。检查时先查看：

```bash
ros2 topic echo /joint_states
```

如果在另一个终端执行，记得先设置 `ROS_DOMAIN_ID=77`。

### 模型只显示部分连杆

检查是否同时运行了多个关节状态发布器，并确认 `/joint_states` 含有当前时间戳。使用本工程的角度输入面板即可正确发布。
