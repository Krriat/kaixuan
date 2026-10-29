#!/usr/bin/env python3
"""
右臂运动学参数提取工具

用法：
    python3 print_kinematics.py          # 输出 URDF 原生参数 + POE 参数
    python3 print_kinematics.py --mdh    # 额外输出 MDH 参数（数值方法近似）

URDF 文件路径：  src/kaixuan1/urdf/kaixuan1.urdf

标定参数更新方法见脚本末尾 update_urdf_joint() 函数。
"""

import sys
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation

URDF_PATH = "/home/ranger/code/new_kaixuan/src/kaixuan1/urdf/kaixuan1.urdf"

RIGHT_ARM_JOINTS = [
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_joint7",
]


# ──────────────────────────── 基础变换工具 ────────────────────────────

def rpy_to_matrix(rpy):
    """URDF RPY 约定：绕固定轴依次旋转 Rx·Ry·Rz（内禀 ZYX / 外禀 XYZ）"""
    return Rotation.from_euler("xyz", rpy).as_matrix()


def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def matrix_to_rpy(R):
    return Rotation.from_matrix(R).as_euler("xyz")


# ──────────────────────────── URDF 解析 ────────────────────────────

def parse_joint(joint_elem):
    origin = joint_elem.find("origin")
    if origin is not None:
        xyz = list(map(float, origin.get("xyz", "0 0 0").split()))
        rpy = list(map(float, origin.get("rpy", "0 0 0").split()))
    else:
        xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    axis_elem = joint_elem.find("axis")
    axis = (
        list(map(float, axis_elem.get("xyz", "0 0 1").split()))
        if axis_elem is not None
        else [0.0, 0.0, 1.0]
    )
    return xyz, rpy, axis


def load_joints(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    return {j.get("name"): j for j in root.findall("joint")}


# ──────────────────────────── 运动学计算 ────────────────────────────

def compute_kinematics(urdf_path, joint_names):
    """
    返回:
        joint_params  : list of (name, xyz, rpy, axis, T_0i)  T_0i = 零位时 base→关节坐标系
        screw_axes    : list of ndarray(6,)  空间坐标系螺旋轴 [ω; v]
        M             : ndarray(4,4)  末端零位位姿（以最后一个关节坐标系为末端）
    """
    joints = load_joints(urdf_path)

    T_current = np.eye(4)
    joint_params = []
    screw_axes = []

    for jname in joint_names:
        j = joints[jname]
        xyz, rpy, axis = parse_joint(j)
        T_joint = make_T(xyz, rpy)
        T_current = T_current @ T_joint

        # POE 空间坐标系螺旋轴（Modern Robotics 第3章惯例）
        omega = T_current[:3, :3] @ np.array(axis)
        omega /= np.linalg.norm(omega)
        p = T_current[:3, 3]               # 轴上一点（关节原点）
        v = -np.cross(omega, p)            # v = -ω × q
        S = np.hstack([omega, v])

        screw_axes.append(S)
        joint_params.append((jname, xyz, rpy, axis, T_current.copy()))

    M = T_current.copy()
    return joint_params, screw_axes, M


# ──────────────────────────── MDH 参数计算 ────────────────────────────

def compute_mdh(joint_params):
    """
    从 URDF 的相邻关节变换中推导 Modified DH 参数（Craig 惯例）。

    MDH 变换：  T_{i-1,i} = Rot_x(α_{i-1}) · Trans_x(a_{i-1}) · Rot_z(θ_i) · Trans_z(d_i)

    在零位（θ=0）时：T = Rot_x(α) · Trans_x(a) · Trans_z(d)
    因此：
        d      = T[2,3]  （在 Rot_x(α)·Trans_x(a) 坐标系中的 z 分量）
        a      = T[0,3] * cos(0) + T[1,3] * sin(0) ...（需数值迭代）

    注意：对于非标准 DH 的机器人，数值结果为最佳近似，可能含残差。
    推荐优先使用 POE 参数（与 URDF 完全等价，无近似）。
    """
    mdh_rows = []

    # 第 0 帧到第 1 帧的变换即为第一个关节的 T
    T_prev = np.eye(4)
    z_prev = np.array([0.0, 0.0, 1.0])  # 前一帧 z 轴（base 系）

    for i, (jname, xyz, rpy, axis, T_0i) in enumerate(joint_params):
        # 当前帧 z 轴（关节轴）
        z_curr = T_0i[:3, :3] @ np.array(axis)
        z_curr /= np.linalg.norm(z_curr)

        # 当前帧 x 轴（公垂线方向）
        cross = np.cross(z_prev, z_curr)
        cross_norm = np.linalg.norm(cross)
        if cross_norm > 1e-6:
            x_curr = cross / cross_norm
        else:
            # 平行轴：x 任意取，与 z_prev 垂直
            x_curr = T_0i[:3, :3] @ np.array([1.0, 0.0, 0.0])
            x_curr -= np.dot(x_curr, z_curr) * z_curr
            x_curr /= np.linalg.norm(x_curr)

        # α_{i-1}：相邻 z 轴夹角
        alpha = np.arctan2(
            np.dot(np.cross(z_prev, z_curr), x_curr),
            np.dot(z_prev, z_curr),
        )

        # a_{i-1}：沿公垂线的距离（相邻 z 轴间距）
        # 找两 z 轴的公垂线距离
        p_prev = T_prev[:3, 3]
        p_curr = T_0i[:3, 3]
        diff = p_curr - p_prev
        if cross_norm > 1e-6:
            a = np.dot(diff, x_curr)
            d = np.dot(diff, z_curr)
        else:
            a = 0.0
            d = np.dot(diff, z_curr)

        # θ_offset：零位时关节角偏置（通常为 0，标定后可能非零）
        theta_offset = 0.0

        mdh_rows.append((jname, alpha, a, d, theta_offset))

        T_prev = T_0i.copy()
        z_prev = z_curr.copy()

    return mdh_rows


# ──────────────────────────── 输出 ────────────────────────────

def print_urdf_params(joint_params):
    print("\n" + "=" * 80)
    print("  URDF 原生关节参数（标定后直接修改这里的数值 → kaixuan1.urdf）")
    print("=" * 80)
    print(f"  {'关节':<16}  {'xyz [m]':^38}  {'rpy [rad]':^38}  {'axis':^22}")
    print("  " + "-" * 76)
    for jname, xyz, rpy, axis, _ in joint_params:
        xyz_s = f"[{xyz[0]:9.6f}, {xyz[1]:9.6f}, {xyz[2]:9.6f}]"
        rpy_s = f"[{rpy[0]:9.6f}, {rpy[1]:9.6f}, {rpy[2]:9.6f}]"
        axis_s = f"[{axis[0]:7.5f},{axis[1]:7.5f},{axis[2]:7.5f}]"
        print(f"  {jname:<16}  {xyz_s}  {rpy_s}  {axis_s}")


def print_poe_params(screw_axes, M):
    print("\n" + "=" * 80)
    print("  POE 参数（空间坐标系，Modern Robotics 惯例）")
    print("  S_i = [ω_x, ω_y, ω_z, v_x, v_y, v_z]  （单位：rad/m，与 URDF 完全等价）")
    print("=" * 80)
    for i, S in enumerate(screw_axes, 1):
        print(
            f"  S{i} = [{S[0]:9.6f}, {S[1]:9.6f}, {S[2]:9.6f},"
            f"  {S[3]:9.6f}, {S[4]:9.6f}, {S[5]:9.6f}]"
        )
    print()
    print("  末端零位位姿 M（base → right_link7，零位构型）：")
    for row in M:
        print("    [" + "  ".join(f"{v:10.6f}" for v in row) + "]")
    rpy_M = matrix_to_rpy(M[:3, :3])
    print(f"\n  M 平移：  xyz = [{M[0,3]:.6f}, {M[1,3]:.6f}, {M[2,3]:.6f}] m")
    print(f"  M 旋转：  rpy = [{rpy_M[0]:.6f}, {rpy_M[1]:.6f}, {rpy_M[2]:.6f}] rad")


def print_mdh_params(mdh_rows):
    print("\n" + "=" * 80)
    print("  Modified DH 参数（Craig 惯例，数值近似——非 DH 结构机器人存在残差）")
    print(f"  {'关节':<16}  {'α_{i-1} [rad]':>16}  {'a_{i-1} [m]':>14}  "
          f"{'d_i [m]':>12}  {'θ_offset [rad]':>16}")
    print("  " + "-" * 76)
    for jname, alpha, a, d, theta in mdh_rows:
        print(f"  {jname:<16}  {alpha:16.6f}  {a:14.6f}  {d:12.6f}  {theta:16.6f}")


# ──────────────────────────── 标定参数回写 ────────────────────────────

def update_urdf_joint(urdf_path, joint_name, new_xyz, new_rpy, new_axis=None,
                      output_path=None):
    """
    将标定后的参数写回 URDF 文件。

    参数：
        urdf_path   : 输入 URDF 路径
        joint_name  : 关节名，如 "right_joint2"
        new_xyz     : [x, y, z]，单位 m
        new_rpy     : [r, p, y]，单位 rad（URDF XYZ 固定轴惯例）
        new_axis    : [ax, ay, az]，None 表示不修改轴方向
        output_path : 输出路径；None 表示原地覆盖

    示例：
        update_urdf_joint(
            URDF_PATH,
            "right_joint2",
            new_xyz=[0.0, -0.06195, 0.059720],
            new_rpy=[1.5740, 1.5708, 0.0],
            output_path=URDF_PATH + ".calibrated",
        )
    """
    if output_path is None:
        output_path = urdf_path

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    found = False
    for j in root.findall("joint"):
        if j.get("name") == joint_name:
            origin = j.find("origin")
            if origin is None:
                origin = ET.SubElement(j, "origin")
            xyz_str = f"{new_xyz[0]:.8f} {new_xyz[1]:.8f} {new_xyz[2]:.8f}"
            rpy_str = f"{new_rpy[0]:.8f} {new_rpy[1]:.8f} {new_rpy[2]:.8f}"
            origin.set("xyz", xyz_str)
            origin.set("rpy", rpy_str)

            if new_axis is not None:
                axis_elem = j.find("axis")
                if axis_elem is None:
                    axis_elem = ET.SubElement(j, "axis")
                axis_str = f"{new_axis[0]:.8f} {new_axis[1]:.8f} {new_axis[2]:.8f}"
                axis_elem.set("xyz", axis_str)

            found = True
            break

    if not found:
        raise ValueError(f"关节 '{joint_name}' 在 URDF 中未找到")

    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="unicode", xml_declaration=True)
    print(f"[update_urdf_joint] 已写入 → {output_path}")


# ──────────────────────────── 主程序 ────────────────────────────

if __name__ == "__main__":
    show_mdh = "--mdh" in sys.argv

    joint_params, screw_axes, M = compute_kinematics(URDF_PATH, RIGHT_ARM_JOINTS)

    print_urdf_params(joint_params)
    print_poe_params(screw_axes, M)

    if show_mdh:
        mdh_rows = compute_mdh(joint_params)
        print_mdh_params(mdh_rows)

    print("\n" + "=" * 80)
    print("  标定参数更新流程")
    print("=" * 80)
    print("""
  【方法 A】直接修改 URDF（最简单，推荐）
  ─────────────────────────────────────────
  文件：src/kaixuan1/urdf/kaixuan1.urdf
  定位每个 <joint name="right_jointX"> 块，修改：
      <origin xyz="x y z" rpy="r p y"/>   ← 平移 (m) 与旋转 (rad，XYZ 固定轴)
      <axis   xyz="ax ay az"/>             ← 轴方向（一般不变）
  修改后执行：
      cd /home/ranger/code/new_kaixuan
      colcon build --base-paths src --packages-select xiaogong_kaixuan kaixuan3_config
  重启机器人节点即可生效。

  【方法 B】用本脚本的 update_urdf_joint() 函数批量写入
  ─────────────────────────────────────────
  在 Python 中调用（可先写到 .calibrated 文件验证再覆盖原文件）：

      from print_kinematics import update_urdf_joint, URDF_PATH

      # 例：将标定后的 right_joint2 参数写入
      update_urdf_joint(
          URDF_PATH,
          joint_name = "right_joint2",
          new_xyz    = [0.0, -0.06195, 0.05972],
          new_rpy    = [1.5740, 1.5708, 0.0],
          output_path = URDF_PATH + ".calibrated",   # 先验证
      )

  【方法 C】若标定给出 MDH 参数 → 先转换再写入
  ─────────────────────────────────────────
  MDH 参数 (a, alpha, d, theta_offset) 需转换为 URDF 的 xyz/rpy。
  由于本机器人不严格满足 DH 约束，建议由标定程序直接输出每个关节的
  位姿矩阵 T_{i-1,i}（4×4），再用 matrix_to_rpy() 提取 xyz/rpy。

  注意：修改 URDF 会同时影响 MoveIt 运动规划与碰撞检测；
        若 kinematics.yaml 中有 KDL/IKFast 专用偏置也需同步更新。
""")
