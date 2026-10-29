# 小工凯旋 ROS 2 工作区

此目录以 `../` 的凯旋工作区为基础，使用 `/home/agxorin/cxl/xiaogong-kaixuan` 的 URDF 和网格，模型包放在 `src/xiaogong_kaixuan`。`kaixuan3_config` 的模拟控制与 MoveIt 配置，以及 `kaixuan_can_hardware` 的 URDF 引用，均已切换到这个模型。源工作区和源模型目录未修改。

模型有左右臂各 7 个活动关节；关节名与原配置相同。新的关节轴向、机械尺寸和限位来自小工模型。MoveIt 的 SRDF 保留规划组和零位状态，只保留相邻连杆的碰撞豁免。

在此目录中构建并启动模拟配置：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch kaixuan3_config moveit_bringup.launch.py use_real_hardware:=false
```

此目录里的 `COLCON_IGNORE` 避免在上层凯旋工作区构建时发现同名包；因此在此目录构建时须使用 `--base-paths src`。

小工 URDF 的 14 个活动关节均写有 `effort=0`、`velocity=0`。现有 `joint_limits.yaml` 为 MoveIt 规划另设 1 rad/s 的速度上限；这不是小工实体硬件的额定参数。复制的 EtherCAT 电机映射、编码器方向、减速比、零位补偿及实际碰撞行为尚未按小工机械臂校准，不能直接用于实体运动。`src/normal` 和 `src/kaixuan_bam` 中的原凯旋脚本也未做硬件适配。
