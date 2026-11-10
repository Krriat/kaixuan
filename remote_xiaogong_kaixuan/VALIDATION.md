# Python 运动接口验证

2026-10-29，Ubuntu 22.04 / ROS 2 Humble。

- `python3 -m unittest discover -s tests -v`：20 项通过，包括默认只规划、执行开关、七关节名称和单位、限位、有限数值、速度比例校验、相对目标、前置检查失败不发送目标、取消应答处理，以及上个目标未结束时阻止新目标。
- 在独立 ROS Domain `193` 启动 `use_real_hardware:=false`、`active_arm:=left` 的 MoveIt 和 `mock_components/GenericSystem`。
- `move_arm.sh --state` 正确读到七个模拟关节的角度。
- `--joint 4 --delta-deg 1` 规划成功，模拟关节保持原位；加 `--execute` 后模拟控制器完成轨迹。
- Python 示例的只规划与执行路径均成功；通过 `degrees=True` 从实时反馈构建关节目标。
- 对未激活的右臂发送请求，被客户端在发送运动目标前拒绝。
- Bash 语法、Python 编译、文件校验清单通过。测试结束后关闭 Domain 193 的模拟节点。

超时测试中，MoveGroup 取消应答曾延迟到模拟轨迹完成之后。客户端对此报告“取消未获确认”，
并提示机械臂可能继续原轨迹；Ctrl+C、超时或客户端退出不能用来确认机械臂已停止。
`ArmClient.cancel()` 返回值只表示取消请求被应答或 action 已结束，不能确认实体停止状态。

本次没有向真实机械臂发送规划或执行目标；两台电脑之间的 DDS 连接和实体运动需要现场验证。

环境脚本退出问题的回归检查：

- 成功和配置错误两条路径，均测试 `errexit` / `nounset` / `pipefail` 的全部 8 种组合，调用前后的 shell 选项一致。
- 普通 shell source 环境后运行一个主动报错的 Python 程序，错误码为 1，后续 shell 命令仍可执行。
- 非法 Domain ID 会使 `move_arm.sh` 子进程退出，调用它的普通 shell 保持运行。
- 修复仅涉及环境脚本、测试和文档；未运行 `--execute` 或向实体发送运动目标。

2026-11-10 客户端模型同步修复：

- 同步机器人当前 URDF、左侧第1～3段网格及第3段独立碰撞网格；17个引用的STL均与机器人源文件逐字节一致。
- 检查和运动接口的模型不一致错误现在列出具体关节字段；仍拒绝不兼容的关节坐标、轴向和限位。
- `python3 -m unittest discover -s tests -v`：25项通过，含右肩偏移差异、Xacro包装兼容、轴向/限位/缺失关节拒绝、资源完整性与源文件同步检查。
- 从AGX本机连接正在运行的Domain26右臂真实硬件bringup，`check_connection.sh --arm right`全部通过，未发送规划或运动目标。该检查不替代用户电脑与机器人的跨机DDS验证。
