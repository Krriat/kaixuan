xiaogong-kaixuan

本模型按要求使用：
  本体：kaixuan\src\tilt_kaixuan\urdf\整体urdf.urdf 中的 base_link
  双臂：峻吉小工-0825.SLDASM\urdf_packages\kaixu_arm 中的原始左右新臂

入口文件：urdf/xiaogong-kaixuan.urdf
请连同 meshes 目录一起使用；压缩包包含完整引用资源。

替换内容
本体网格、惯量以及 world_to_base 固定变换取自 tilt_kaixuan。
删除原小工本体的 base_link、腰部、头部及其关节。
两侧机械臂的全部网格、惯量、关节轴向、限位、内部关节变换及末端坐标直接保留 kaixu_arm 原文件数据。
只修改 left_joint1 / right_joint1 的 parent 和 origin，使新臂接到凯旋本体的倾斜肩座。肩部原点和轴线取凯旋模型，绕安装轴调整局部朝向，使新臂在零位保持下垂：左侧 -90 度、右侧 180 度。
源文件和源文件夹均未修改。未使用上一轮 junji_dual_7_6 重建包中的双臂网格或参数。

模型结构
18 个连杆，17 个关节；左右臂各 7 个活动关节，共 14 个活动关节。
根节点 world；world_to_base 保留原模型的 xyz="0 0 1"。
长度 m，角度 rad，质量 kg，惯量 kg*m^2。
对外模型名、文件名和打包目录为 xiaogong-kaixuan。
ROS 包名为 xiaogong_kaixuan，使用下划线以符合 ROS 包命名规则。

ROS 2 Humble / RViz2
将整个 xiaogong-kaixuan 文件夹放入工作区 src 目录：
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select xiaogong_kaixuan
  source install/setup.bash
  ros2 launch xiaogong_kaixuan display.launch.py

不安装 ROS 的本机预览
在本包目录运行：
  python preview.py
浏览器会打开 http://127.0.0.1:8767/preview/ 。
支持旋转、缩放、逐关节调整及零位/弯曲姿态切换。

参数说明
本次只做本体替换和肩部安装坐标适配。双臂 effort/velocity=0、限位、末端参考几何和惯量均沿用 kaixu_arm 原模型；没有套用另一套手臂的控制器、电机映射或零位偏置。
安装角度按两个 URDF 的坐标和网格适配，未核验真实安装孔及机械适配件；本包不代表真实硬件装配验证。
ROS 2 启动文件已做语法检查；当前 Windows 上未运行 ROS 2/RViz2。

资料
docs/source_manifest.json：两个模型的来源、SHA256、资源文件清单和肩部变换。
docs/validation.json：模型结构、源数据保留、网格路径与资源完整性检查。
docs/joints.csv：全部关节与安装参数。
preview_zero.png / preview_bent.png：实际加载本 URDF 的预览截图。
preview/vendor：预览库及其原许可证。

2026-11-03 本地模型调整
相对原始模型，left_joint1 沿自身 Z 轴累计调整 +180 度，left_joint2 累计调整 +190 度，left_joint5 调整 +180 度，right_joint2 调整 +10 度。调整写入关节 origin 的旋转，不修改关节 axis、限位或硬件编码器补偿。remote_xiaogong_kaixuan/model/robot.urdf 同步使用相同几何。

零位轴线校正：以 world/base_link 为基准，left_joint2 为 +X 水平轴，left_joint3 为 -Z 竖直轴，left_joint4 为 -Y 水平轴，left_joint5 为 -Z 竖直轴，left_joint6 为 +Y 水平轴（2026-11-04 修正）。已通过微小 origin 旋转校正原始角度小数近似导致的偏斜；报告见 docs/left_arm_axis_alignment.json。

左臂当前方向：两份 URDF 的 left_joint2 轴向为 0 0 -1；left_joint6 于 2026-11-04 从 0 0 -1 修正为 0 0 1。此前 origin 旋转角记录以原始 +Z 轴为基准。硬件方向系数均为 +1，避免重复反向。

2026-11-03 第5轴角度表示更新：left_joint5 的位置范围由近似 ±3.14 rad 改为精确 ±π。硬件编码器偏置增加一整圈，反馈归一化到 [-180°, 180°)，原 180.018° 对应约 -179.982°，几何和实体姿态保持等价。当前模型与 remote 模型的第5轴限位同步。

2026-11-04 第6轴方向修正：left_joint6 的 axis 从 0 0 -1 改为 0 0 1，本地和 remote 模型同步。URDF 零位下该轴为 +Y 水平方向；硬件方向系数仍为 +1，编码器偏置不变。重力补偿读取新 URDF 重新计算全部关节力矩。

2026-11-10 右侧参考模型统一
右肩 right_joint1 的 origin X 从 0.010173 m 改为 0；安装点现在为 (0, -0.20995, -0.049457)。
左侧第1、2、3连杆改用右侧对应模型：第1段沿局部X镜像，第2段绕局部Y旋转180度，第3段绕局部Z旋转180度。
显示和碰撞网格均按同一变换生成；反射网格已反转三角面绕序。第3段保留右侧显示/碰撞模型的区别，左侧碰撞文件为 left_link3_collision_from_right.STL。
质量与右侧一致，质心按 c_left=A*c_right 转换，质心处惯量按 I_left=A*I_right*A^T 转换。
原关节坐标、轴向、编码器标定和下游关节连接原点保持不变；本次是用户指定的模型统一，不是新的实体质量/惯量辨识。
变换和来源哈希见 docs/left_links_from_right.json；此前原始模型保留说明作为历史记录。
