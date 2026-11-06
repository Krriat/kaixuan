"""Replay logged physics; plotting/rendering never generates new controller results."""
import argparse
import json
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

from .model import build_model


def connector(scene, start, end, rgba, *, arrow=False, width=.002):
    if scene.ngeom >= scene.maxgeom or np.linalg.norm(np.asarray(end)-start) < 1e-9:
        return
    geom = scene.geoms[scene.ngeom]
    kind = mujoco.mjtGeom.mjGEOM_ARROW if arrow else mujoco.mjtGeom.mjGEOM_CAPSULE
    mujoco.mjv_initGeom(geom, kind, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.asarray(rgba, dtype=np.float32))
    mujoco.mjv_connector(geom, kind, width, np.asarray(start), np.asarray(end))
    scene.ngeom += 1


def overlay(scene, desired, actual, index, force, *, force_arrow_scale=.05):
    for k in range(0, len(desired)-1, 10):
        connector(scene, desired[k], desired[min(k+10, len(desired)-1)], [0.1, .85, .35, 1.])
    for k in range(0, index, 10):
        connector(scene, actual[k], actual[min(k+10, index)], [.2, .5, 1., 1.], width=.0015)
    # Visualization-only markers. These do not alter the simulated robot.
    for pos, size, color in ((desired[index], .009, [0.1, .85, .35, 1.]),
                             (actual[index], .011, [.15, .45, 1., 1.])):
        if scene.ngeom < scene.maxgeom:
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                np.full(3, size), pos, np.eye(3).ravel(), np.asarray(color, dtype=np.float32))
            scene.ngeom += 1
    if np.linalg.norm(force) > .01:
        connector(scene, actual[index], actual[index]+force_arrow_scale*force, [1., .12, .08, 1.], arrow=True, width=.008)


def replay(output, config, settings, *, video=False, viewer=False, case="push_tvlqr"):
    output = Path(output)
    with np.load(output / case / "trace.npz") as saved:
        trace = {key: saved[key] for key in saved.files}
    path = build_model(config, output / "replay_model.xml", visuals=True)
    model = mujoco.MjModel.from_xml_path(str(path))
    model.vis.headlight.ambient[:] = [.45, .45, .45]
    model.vis.headlight.diffuse[:] = [.75, .75, .75]
    model.vis.headlight.specular[:] = [.15, .15, .15]
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    presentation = settings.get("presentation", {})
    camera.lookat[:] = presentation.get("camera_lookat_world_m", [.18, -.1, .65])
    camera.distance = presentation.get("camera_distance_m", 1.4)
    camera.azimuth = presentation.get("camera_azimuth_deg", 90.)
    camera.elevation = presentation.get("camera_elevation_deg", -15.)
    # Make the moving left arm easy to identify; static geometry remains visible
    # but translucent. Rendering colors never change physical parameters.
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] != 0:
            model.geom_rgba[geom] = [.30, .60, .90, 1.]
        elif model.geom(geom).name != "ground_visual":
            model.geom_rgba[geom] = [.55, .60, .66, .12]
    times = trace["time_s"]
    desired, actual = trace["desired_position_m"], trace["ee_position_m"]
    def frame(index, scene):
        data.qpos[:] = trace["q_rad"][index]
        data.qvel[:] = trace["dq_rad_s"][index]
        data.time = times[index]
        mujoco.mj_forward(model, data)
        force = trace["force_world_n"][min(index, len(times)-2)]
        overlay(scene, desired, actual, index, force,
                force_arrow_scale=presentation.get("force_arrow_m_per_n", .05))
    if video:
        from .lqr_video import render_explained_video
        render_explained_video(output, case, trace, settings, model, data, camera, frame)
    if viewer:
        from mujoco import viewer as mj_viewer
        existing_threads = set(threading.enumerate())
        window = mj_viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
        owned_threads = set(threading.enumerate())-existing_threads
        try:
            with window:
                with window.lock():
                    window.cam.lookat[:] = camera.lookat
                    window.cam.distance, window.cam.azimuth, window.cam.elevation = camera.distance, camera.azimuth, camera.elevation
                start = time.perf_counter()
                speed = presentation.get("playback_speed", 1.)
                for t in np.arange(0., times[-1], speed/60):
                    if not window.is_running():
                        break
                    index = min(int(np.searchsorted(times, t)), len(times)-1)
                    with window.lock():
                        window.user_scn.ngeom = 0
                        frame(index, window.user_scn)
                    window.sync()
                    time.sleep(max(0., start+t/speed+1/60-time.perf_counter()))
        finally:
            # MuJoCo 3.3.7 close() requests shutdown asynchronously. Wait for
            # the threads this launch created before Python/GLFW atexit runs;
            # otherwise GLX may destroy a drawable while it is still rendering.
            window.close()
            for thread in owned_threads:
                thread.join(timeout=5.)
            if any(thread.is_alive() for thread in owned_threads):
                raise RuntimeError("MuJoCo replay window did not shut down cleanly")
        print("Viewer replay closed cleanly", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--case", default="push_tvlqr")
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    summary = json.loads((args.output / "summary.json").read_text())
    replay(args.output, summary["controller_model_config"], summary["settings"],
           case=args.case, video=args.video, viewer=not args.video)


if __name__ == "__main__":
    main()
