xiaogong-kaixuan

本模型按要求使用：
  本体：kaixuan\src\tilt_kaixuan\urdf\整体urdf.urdf 中的 base_link
  双臂：峻吉小工-0825.SLDASM\urdf_packages\kaixu_arm 中的原始左右新臂

入口文件：urdf/xiaogong-kaixuan.urdf
请连同 meshes 目录一起使用；压缩包包含完整引用资源。

替换内容
本体网格、惯量以及 world_to_base 固定变换取自 tilt_kaixuan。
删除原小工本体的 base_link、腰部、头部及其关节。
两侧机械臂的全部网格、惯量、关节轴向、限位及末端坐标保留 kaixu_arm 原文件数据；左臂七个关节的 origin 旋转已按下述零位校正。
最初修改 left_joint1 / right_joint1 的 parent 和 origin，使新臂接到凯旋本体的倾斜肩座。肩部原点和轴线取凯旋模型，绕安装轴调整局部朝向，使新臂在零位保持下垂：左侧 -90 度、右侧 180 度。
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
本模型包含本体替换、肩部安装坐标适配和左臂几何零位校正。双臂 effort/velocity=0、限位、末端参考几何和惯量均沿用 kaixu_arm 原模型；没有套用另一套手臂的控制器、电机映射或零位偏置。
安装角度按两个 URDF 的坐标和网格适配，未核验真实安装孔及机械适配件；本包不代表真实硬件装配验证。
ROS 2 启动文件已做语法检查；当前 Windows 上未运行 ROS 2/RViz2。

资料
docs/source_manifest.json：两个模型的来源、SHA256、资源文件清单和肩部变换。
docs/validation.json：模型结构、源数据保留、网格路径与资源完整性检查。
docs/joints.csv：全部关节与安装参数。
preview_zero.png / preview_bent.png：实际加载本 URDF 的预览截图。
preview/vendor：预览库及其原许可证。

2026-10-29 左臂零位校正
按用户 RViz 截图，将原模型 left_joint1..7 的 [-180, -180, -7, 39, 180, -17, -6] 度姿态设为新模型全零位。
各关节 R_new_origin = R_old_origin * Rz(offset)，新模型 FK(q) = 原模型 FK(q + offset)。
机器人端与 remote_xiaogong_kaixuan/model/robot.urdf 同步；右臂、关节轴、限位及硬件编码器补偿沿用现有配置。
原始 origin 和校正角度见 docs/source_manifest.json 的 left_zero_calibration。实体姿态仍待现场核对。
