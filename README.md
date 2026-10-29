# 小工凯旋

此仓库包含两个目录：

- [`xiaogong_kaixuan/`](xiaogong_kaixuan/README.md)：ROS 2 Humble 工作区，含小工凯旋 URDF、MoveIt 配置和复制的 EtherCAT 驱动。
- [`remote_xiaogong_kaixuan/`](remote_xiaogong_kaixuan/README.md)：独立的远程 RViz 客户端，内含匹配的模型和网格。

构建工作区：

```bash
cd /home/agxorin/cxl/kaixuan/xiaogong_kaixuan
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

已完成构建、URDF 与网格校验、模拟控制器启动，以及左臂 7 个 EtherCAT 驱动的**只读**反馈检查。实体运动没有执行或验证；详情见 [验证记录](xiaogong_kaixuan/VALIDATION.md)。
