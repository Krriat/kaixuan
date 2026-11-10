#!/usr/bin/env python3
"""原生 OMPL PRM 预建、保存与查询；本工具始终只规划，不执行机械臂。"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid

from geometry_msgs.msg import Vector3
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, SetPlannerParams
from rcl_interfaces.srv import GetParameters
from rclpy.serialization import serialize_message
from rosidl_runtime_py.convert import message_to_ordereddict

from ompl_prm import PARAMETERS
from planning_common import (FRAME, GROUP, JOINTS, LeftArmPlanner, error_text, joint_goal,
                             rclpy, ros_environment_text, validate_trajectory)

# ===== 路网参数；查询终点、速度与加速度继续修改 ompl_prm.py =====
BUILD_TIME_S = 10.0
COLLISION_RESOLUTION = 0.001  # Prebuilt edges use finer collision sampling than the original demos.
CACHE_ROOT = Path(__file__).parent / "results" / "prm_roadmaps"
NATIVE_BUILD = Path(__file__).parent / "results" / "prm-native-build"


def scene_payload(scene):
    """Ignore left-arm position/time; retain geometry, ACM, attached objects and other joints."""
    value = copy.deepcopy(scene)
    state = value.robot_state.joint_state
    fixed = sorted((name, position) for name, position in zip(state.name, state.position)
                   if name not in JOINTS)
    state.name = [pair[0] for pair in fixed]
    state.position = [pair[1] for pair in fixed]
    state.velocity = []
    state.effort = []
    value.name = ""
    value.object_colors = []
    def clean(data):
        if isinstance(data, dict):
            return {key: clean(item) for key, item in data.items() if key != "stamp"}
        if isinstance(data, (list, tuple)):
            return [clean(item) for item in data]
        return data
    return clean(message_to_ordereddict(value))


def capture_context(node):
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    request = GetPlanningScene.Request()
    request.components.components = 1023  # All PlanningSceneComponents in Humble.
    scene = node.call(client, request).scene
    node.destroy_client(client)
    client = node.create_client(GetParameters, "/move_group/get_parameters")
    response = node.call(client, GetParameters.Request(names=["robot_description", "robot_description_semantic"]))
    node.destroy_client(client)
    urdf, srdf = [value.string_value for value in response.values]
    if not urdf or not srdf:
        raise RuntimeError("Cannot read MoveIt's robot description and semantic model")
    identity = {"scene": scene_payload(scene), "urdf_sha256": hashlib.sha256(urdf.encode()).hexdigest(),
                "srdf_sha256": hashlib.sha256(srdf.encode()).hexdigest(), "group": GROUP,
                "max_nearest_neighbors": PARAMETERS["planner_parameters"]["max_nearest_neighbors"],
                "collision_resolution": COLLISION_RESOLUTION}
    fingerprint = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    return {"fingerprint": fingerprint, "folder": CACHE_ROOT / fingerprint,
            "identity": identity, "scene": scene, "urdf": urdf, "srdf": srdf,
            "start": node.snapshot()}


def motion_request(start, planner_id):
    goal = PARAMETERS["joint_goal_deg"]
    if (not isinstance(goal, (list, tuple)) or len(goal) != 7
            or not all(type(value) in (int, float) and math.isfinite(value) for value in goal)):
        raise ValueError("Fill ompl_prm.py TARGET_JOINTS_DEG with seven finite angles")
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.group_name = GROUP
    motion.pipeline_id = "ompl"
    motion.planner_id = planner_id
    motion.start_state = copy.deepcopy(start)
    motion.goal_constraints = [joint_goal([math.radians(value) for value in goal])]
    motion.allowed_planning_time = float(PARAMETERS["planning_time"])
    motion.num_planning_attempts = 1
    motion.max_velocity_scaling_factor = float(PARAMETERS["velocity_scale"])
    motion.max_acceleration_scaling_factor = float(PARAMETERS["acceleration_scale"])
    motion.workspace_parameters.header.frame_id = FRAME
    motion.workspace_parameters.min_corner = Vector3(x=-2., y=-2., z=-2.)
    motion.workspace_parameters.max_corner = Vector3(x=2., y=2., z=2.)
    return request


def ensure_builder():
    source = Path(__file__).parent / "prm_roadmap_native"
    NATIVE_BUILD.mkdir(parents=True, exist_ok=True)
    commands = [["cmake", "-S", str(source), "-B", str(NATIVE_BUILD), "-DCMAKE_BUILD_TYPE=Release"],
                ["cmake", "--build", str(NATIVE_BUILD), "-j2"]]
    with (NATIVE_BUILD / "build.log").open("w") as log:
        for command in commands:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
            if completed.returncode:
                raise RuntimeError(f"Native builder compilation failed; see {NATIVE_BUILD / 'build.log'}")
    return NATIVE_BUILD / "prm_roadmap_builder"


def build_map(node, context, seconds=BUILD_TIME_S):
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("BUILD_TIME_S must be finite and positive")
    executable = ensure_builder()
    folder = context["folder"]
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "robot.urdf").write_text(context["urdf"])
    (folder / "robot.srdf").write_text(context["srdf"])
    (folder / "scene.cdr").write_bytes(serialize_message(context["scene"]))
    (folder / "request.cdr").write_bytes(serialize_message(motion_request(context["start"], "PRM").motion_plan_request))
    (folder / "identity.json").write_text(json.dumps(context["identity"], indent=2) + "\n")
    environment = dict(os.environ, ROS_DOMAIN_ID="130", ROS_LOCALHOST_ONLY="1")
    with (folder / "builder.log").open("w") as log:
        completed = subprocess.run([str(executable), str(folder), str(seconds),
            str(PARAMETERS["planner_parameters"]["max_nearest_neighbors"]), str(COLLISION_RESOLUTION)],
            env=environment, stdout=log, stderr=subprocess.STDOUT)
    if completed.returncode:
        raise RuntimeError(f"Roadmap build failed; see {folder / 'builder.log'}")
    after = capture_context(node)
    if after["fingerprint"] != context["fingerprint"]:
        raise RuntimeError("Planning scene changed during build; capture and build the current scene again")
    return json.loads((folder / "build_result.json").read_text())


def query_map(node, context, prebuilt=True, simplify=True):
    folder = context["folder"]
    if prebuilt and not (folder / "roadmap.graph").is_file():
        raise RuntimeError("No map for this exact scene/model/fixed-joint state; run 'prm_roadmap.py build' first")
    # Load the immutable, explicitly weighted map into a fresh query planner. This also
    # avoids Humble's cached-graph reconstruction/export changing existing edge costs.
    ident = ("PRMDisk_" if prebuilt else "PRMCold_") + context["fingerprint"][:12] + "_" + uuid.uuid4().hex[:8]
    values = {"type": "geometric::PRM", "max_nearest_neighbors": str(PARAMETERS["planner_parameters"]["max_nearest_neighbors"]),
              "longest_valid_segment_fraction": str(COLLISION_RESOLUTION),
              "multi_query_planning_enabled": "1" if prebuilt else "0",
              "load_planner_data": "1" if prebuilt else "0", "store_planner_data": "0",
              "simplify_solutions": "1" if simplify else "0"}
    if prebuilt:
        values["planner_data_path"] = str(folder / "roadmap.graph")
    config = SetPlannerParams.Request(pipeline_id="ompl", group=GROUP, planner_config=ident, replace=True)
    config.params.keys, config.params.values = list(values), list(values.values())
    node.call(node.set_params_client, config)
    node.planner_interfaces = None
    node.require_planner("ompl", ident)
    request = motion_request(context["start"], ident)
    before = time.monotonic()
    response = node.call(node.plan_client, request, PARAMETERS["planning_time"] + 15).motion_plan_response
    result = {"planner_id": ident, "prebuilt": prebuilt, "simplify": simplify,
              "scene_fingerprint": context["fingerprint"], "planning_s": response.planning_time,
              "planning_service_wall_s": time.monotonic() - before, "error_code": response.error_code.val,
              "execute": False, "roadmap_file": str(folder / "roadmap.graph") if prebuilt else None}
    if response.error_code.val != 1:
        result["error"] = error_text(response.error_code.val)
        return result, None
    result.update(trajectory_duration_s=validate_trajectory(response.trajectory),
                  trajectory_points=len(response.trajectory.joint_trajectory.points))
    if capture_context(node)["fingerprint"] != context["fingerprint"]:
        raise RuntimeError("Scene changed during query; discard this result and rebuild for the new scene")
    return result, response.trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=["build", "query"])
    args = parser.parse_args()
    rclpy.init(args=[])
    node = LeftArmPlanner()
    try:
        print(ros_environment_text(), flush=True)
        context = capture_context(node)
        print(f"Map for this scene: {context['folder']}", flush=True)
        if args.operation == "build":
            result = build_map(node, context)
        else:
            result, _ = query_map(node, context)
        print(json.dumps(result, indent=2))
        return 0 if args.operation == "build" or result["error_code"] == 1 else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
