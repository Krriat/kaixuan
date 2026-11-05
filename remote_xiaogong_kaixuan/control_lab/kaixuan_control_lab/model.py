"""Convert this robot's fixed/revolute URDF tree to an explicit MJCF model.

Inertial tensors are taken from URDF, never inferred from visual meshes.
This intentionally small converter rejects unsupported joints/geometries.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .config import DEFAULT_CONFIG, ROOT, load_config, urdf_path


def numbers(values):
    return " ".join(format(float(value), ".17g") for value in values)


def pose(element):
    origin = element.find("origin")
    xyz = np.zeros(3) if origin is None else np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
    rpy = np.zeros(3) if origin is None else np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
    roll, pitch, yaw = rpy / 2
    cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
    # URDF Rz(yaw) Ry(pitch) Rx(roll); MuJoCo quaternion order is w,x,y,z.
    quat = np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])
    w, x, y, z = quat
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return xyz, quat, rotation


def build_model(config, output, *, visuals=True):
    output = Path(output).resolve()
    source = urdf_path(config)
    robot = ET.parse(source).getroot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    children = {}
    joint_by_name = {}
    child_names = set()
    for joint in robot.findall("joint"):
        if joint.attrib["type"] not in ("fixed", "revolute") or joint.find("mimic") is not None:
            raise ValueError(f"Unsupported joint: {joint.attrib}")
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        children.setdefault(parent, []).append(joint)
        joint_by_name[joint.attrib["name"]] = joint
        child_names.add(child)
    roots = set(links) - child_names
    if len(roots) != 1:
        raise ValueError("Expected a single fixed-base URDF tree")
    active = config["joint_names"]
    for name in active:
        if name not in joint_by_name or joint_by_name[name].attrib["type"] != "revolute":
            raise ValueError(f"Unknown revolute joint: {name}")
    estimates = config["inertia"]["armature_kg_m2"]
    if set(estimates) - set(joint_by_name):
        raise ValueError("armature contains an unknown joint name")
    if config["end_effector_frame"] not in links:
        raise ValueError("Unknown end effector frame")
    mjcf = ET.Element("mujoco", model="kaixuan_control_lab")
    ET.SubElement(mjcf, "compiler", angle="radian", inertiafromgeom="false",
                  fusestatic="true")
    ET.SubElement(mjcf, "option", timestep=str(config["physics_step_s"]),
                  gravity=numbers(config["gravity_m_s2"]), integrator="implicitfast")
    asset = ET.SubElement(mjcf, "asset")
    world = ET.SubElement(mjcf, "worldbody")
    ET.SubElement(world, "light", pos="1 -1 3", dir="-1 1 -2")
    ET.SubElement(world, "geom", name="ground_visual", type="plane", size="2 2 0.1",
                  contype="0", conaffinity="0", rgba="0.25 0.28 0.32 1")
    meshes = {}
    visited = []

    def add_link(name, parent, joint=None):
        link = links[name]
        attrs = {"name": "urdf_world" if name == "world" else name}
        if joint is not None:
            xyz, quat, _ = pose(joint)
            attrs.update(pos=numbers(xyz), quat=numbers(quat))
        body = ET.SubElement(parent, "body", attrs)
        if joint is not None and joint.attrib["name"] in active:
            jname = joint.attrib["name"]
            index = active.index(jname)
            visited.append(jname)
            limits = joint.find("limit")
            ET.SubElement(body, "joint", name=jname, type="hinge", pos="0 0 0",
                          axis=joint.find("axis").get("xyz"), limited="true",
                          range=f'{limits.get("lower")} {limits.get("upper")}',
                          armature=str(estimates.get(jname, 0.0)),
                          damping=str(config["physical_friction"]["viscous_nm_s_rad"][index]),
                          frictionloss=str(config["physical_friction"]["coulomb_nm"][index]))
        inertial = link.find("inertial")
        if inertial is not None:
            xyz, _, rotation = pose(inertial)
            inertia = inertial.find("inertia").attrib
            xx, yy, zz, xy, xz, yz = (float(inertia[key]) for key in
                                      ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))
            tensor = rotation @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ rotation.T
            ET.SubElement(body, "inertial", pos=numbers(xyz),
                          mass=inertial.find("mass").get("value"),
                          fullinertia=numbers([tensor[0, 0], tensor[1, 1], tensor[2, 2],
                                               tensor[0, 1], tensor[0, 2], tensor[1, 2]]))
        if name == config["end_effector_frame"]:
            ET.SubElement(body, "site", name="ee_site", size="0.008", rgba="1 0.2 0.1 1")
        if visuals:
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    raise ValueError(f"Unsupported visual on {name}")
                uri = mesh.attrib["filename"]
                prefix = "package://xiaogong_kaixuan/"
                if not uri.startswith(prefix):
                    raise ValueError(f"Unsupported mesh URI: {uri}")
                path = source.parent / uri[len(prefix):]
                if not path.is_file():
                    raise FileNotFoundError(path)
                key = (uri, mesh.get("scale", "1 1 1"))
                if key not in meshes:
                    mesh_name = f"mesh_{len(meshes)}"
                    meshes[key] = mesh_name
                    ET.SubElement(asset, "mesh", name=mesh_name,
                                  file=os.path.relpath(path, output.parent), scale=key[1])
                xyz, quat, _ = pose(visual)
                color = visual.find("material/color")
                ET.SubElement(body, "geom", type="mesh", mesh=meshes[key],
                              pos=numbers(xyz), quat=numbers(quat), group="2", density="0",
                              contype="0", conaffinity="0",
                              rgba=color.get("rgba") if color is not None else "0.7 0.7 0.7 1")
        for child_joint in children.get(name, []):
            add_link(child_joint.find("child").get("link"), body, child_joint)

    add_link(next(iter(roots)), world)
    if visited != active:
        raise ValueError("joint_names must follow the selected URDF traversal order")
    actuators = ET.SubElement(mjcf, "actuator")
    for name, limit in zip(active, config["limits"]["torque_nm"]):
        ET.SubElement(actuators, "motor", name=f"{name}_torque", joint=name, gear="1",
                      ctrllimited="true", ctrlrange=numbers([-limit, limit]))
    keyframes = ET.SubElement(mjcf, "keyframe")
    ET.SubElement(keyframes, "key", name="initial", qpos=numbers(config["initial_position_rad"]))
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(mjcf)
    ET.ElementTree(mjcf).write(output, encoding="utf-8", xml_declaration=True)
    metadata = {"source_urdf_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "config": config, "visual_meshes": visuals,
                "frozen_joint_position_rad": {name: 0.0 for name, joint in joint_by_name.items()
                    if joint.get("type") == "revolute" and name not in active}}
    output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/left_arm.xml")
    parser.add_argument("--no-visuals", action="store_true")
    args = parser.parse_args()
    path = build_model(load_config(args.config), args.output, visuals=not args.no_visuals)
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(path))
    print(f"Compiled {path}: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    print(f"Output-side armature (kg.m^2): {model.dof_armature}")


if __name__ == "__main__":
    main()
