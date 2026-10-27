# Kaixuan KGU BAM 模型

这里提供当前双臂 14 个关节对应的 3 份 BAM（Better Actuator Models）
可运行初版：

| BAM 名称 | 关节 | 规格 | 额定/峰值转矩 | 额定/最高速度 |
| --- | --- | --- | ---: | ---: |
| `kgu20_c_48v` | 两侧 J1–J2 | KGU-20、101:1、48 V | 68/126 Nm | 30/40 rpm |
| `kgu17_b_48v` | 两侧 J3–J4 | KGU-17、81:1、48 V | 40/91 Nm | 38/62 rpm |
| `kgu11_c_48v` | 两侧 J5–J7 | KGU-11、101:1、48 V | 15/30 Nm | 32/50 rpm |

完整关节映射在
[`dual_arm_joint_map.json`](kaixuan_bam/config/dual_arm_joint_map.json)。

## 模型可信度

这些文件是 **bootstrap M1**，目的是先让 MuJoCo/mjlab 仿真使用正确量级的
转矩–转速包络，不是已经用摆锤台辨识过的正式 M1/M6：

- `kt`、`R` 是根据 48 V、额定转矩/速度、最高空载速度得到的输出侧等效值；
- `current_limit` 令堵转转矩不超过公开峰值转矩；
- `armature` 暂用整机绕轴几何惯量估计，不等同于真实转子折算惯量；
- `friction_base` 暂取额定转矩的 1%；
- `friction_viscous` 暂取额定转速下额定转矩的 0.5%；
- `kp_joint`、`kd_joint` 是稳定仿真的工程初值；
- `command_delay=1 ms` 对应当前 EtherCAT 周期。

所以它适合联调、控制器开发和训练管线冒烟测试；要做可信 Sim2Real，必须针对
每个“型号 + 减速比”做 BAM 摆锤辨识，并用留出轨迹验证。

## 环境

BAM 1.0.2 要求 Python 3.12。当前 ROS 2 Humble 系统 Python 是 3.10，建议给
仿真单独建 Python 3.12 环境，不要替换 ROS 的系统 Python。

```bash
cd /home/agxorin/cxl/kaixuan/src/kaixuan_bam
python3.12 -m venv .venv
source .venv/bin/activate
pip install -e .
python scripts/validate_models.py
```

MuJoCo CPU 还需安装 BAM 的 `mujoco` extra；mjlab 使用 `mjlab` extra。

## MuJoCo CPU 用法

MJCF 中每个关节使用 `motor` 且 `gear="1"`。模型已经是输出轴等效参数，不能
再次乘 81 或 101：

```xml
<actuator>
  <motor name="right_joint1" joint="right_joint1" gear="1"/>
</actuator>
```

```python
from kaixuan_bam import load_kgu_model
from bam.mujoco import MujocoController

model = load_kgu_model("kgu20_c_48v")
controller = MujocoController(
    model=model,
    actuator=["right_joint1", "right_joint2"],
    mujoco_model=mj_model,
    mujoco_data=mj_data,
)
```

同一份模型可以驱动同规格的多个关节；三种规格应分别建立 controller。

## mjlab 用法

先导入 `kaixuan_bam` 完成自定义执行器注册，然后把 JSON 的绝对路径交给 BAM：

```python
import kaixuan_bam  # 必须先导入
from bam.mjlab import make_bam_actuator_cfg

cfg = make_bam_actuator_cfg(
    json_path=(
        "/home/agxorin/cxl/kaixuan/src/kaixuan_bam/kaixuan_bam/"
        "params/kgu20_c_48v/bootstrap_m1.json"
    ),
    target_names_expr=(r"(?:right|left)_joint[12]",),
)
```

另外两组分别匹配 J3–J4 和 J5–J7。

## 现场只读核对结果（2026-09-15）

- 两侧 J1–J2：`Pn085=330`，即额定电流 3.3 A；
- J3–J4：多数为 2.5 A，`left_joint3` 为 2.0 A，拟合前需核实铭牌；
- 两侧 J5–J7：额定电流 1.0 A；
- J1–J4 `Pn080=8`，J5–J7 `Pn080=7`；V1.2 手册写 Pn080 只支持
  4/5/8，与现场值不一致；
- 所有关节 `Pn083=0`，`0x6077` 是额定转矩的 0.1%；
- 所有关节 `Pn088=0`，`0x606C` 是 count/s，并非 rpm。

最后一项很关键：当前
`kaixuan_can_hardware` 将 `velocity_raw` 按 rpm 转换，因此现有 flight recorder
里的 `velocity_rad_s` / `velocity_deg_s` 不适合作为 BAM 拟合输入。正式采数前应
先统一 Pn088 与驱动转换逻辑。

## 正式辨识还缺什么

每种规格至少准备 3 组摆臂长度 × 3 组端部质量，记录 BAM 的四种 6 秒激励轨迹，
并扫 4 个安全的位置环增益。每条样本至少要有：时间、目标角、输出角、输出速度、
使能状态、实际转矩/电流；`lift_and_drop` 需要安全实现撤力段。采集必须在单关节
牢固固定、有机械限位和急停的测试台上进行，不能直接让装好的双臂自由跌落。

完成数据采集后，依次拟合 M1–M6，并按留出的增益/负载轨迹选择验证误差最低且
参数不过度复杂的模型。正式结果应另存为 `m1.json` … `m6.json`，保留这里的
`bootstrap_m1.json` 作为基准。
