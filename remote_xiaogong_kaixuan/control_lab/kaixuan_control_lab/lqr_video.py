"""Annotated scientific replay: exact robot poses plus synchronized force/error plots."""
import json
from pathlib import Path
import shutil
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from matplotlib.font_manager import FontProperties, findfont


BG = (242, 245, 250)
INK = (25, 40, 63)
BLUE = (36, 99, 235)
GREEN = (20, 151, 92)
RED = (218, 45, 54)
GRAY = (140, 151, 166)


def projection_basis(displacement, force):
    along = np.array(displacement, dtype=float, copy=True)
    along /= np.linalg.norm(along)
    lateral = np.asarray(force, dtype=float)-np.dot(force, along)*along
    if np.linalg.norm(lateral) < 1e-8:
        axis = np.eye(3)[np.argmin(np.abs(along))]
        lateral = axis-np.dot(axis, along)*along
    return np.vstack((along, lateral/np.linalg.norm(lateral)))


def phase_caption(t, settings):
    pulse, trajectory = settings["disturbance"], settings["trajectory"]
    if pulse["start_s"] <= t < pulse["start_s"]+pulse["duration_s"]:
        return "正在向侧面推！", RED
    if t >= pulse["start_s"]+pulse["duration_s"]:
        if t < trajectory["start_hold_s"]+trajectory["move_s"]:
            return "已撤去外力，继续沿直线前进", GREEN
        return "到达终点，保持位置", GREEN
    if t < trajectory["start_hold_s"]:
        return "准备：绿线是接下来要走的路径", BLUE
    return f"直线运动中，{pulse['start_s']:.2f} 秒开始侧推", BLUE


class ExplainedFrame:
    """Compose original-scale 3-D replay and quantitative plots; no pose exaggeration."""
    width, height = 1600, 900
    render_width, render_height = 900, 794

    def __init__(self, trace, settings, nominal=None, *, controller_label="时变 LQR", status="completed"):
        self.trace, self.settings, self.nominal = trace, settings, nominal
        self.controller_label, self.status = controller_label, status
        font_file = findfont(FontProperties(family=["Noto Sans CJK SC", "Noto Sans CJK JP", "sans-serif"]))
        self.fonts = {size: ImageFont.truetype(font_file, size=size) for size in (17, 19, 21, 24, 28, 32)}
        self.t = trace["time_s"]
        self.desired, self.actual = trace["desired_position_m"], trace["ee_position_m"]
        self.error = 1000*np.linalg.norm(self.actual-self.desired, axis=1)
        self.force = np.linalg.norm(trace["force_world_n"], axis=1)
        displacement = np.asarray(settings["trajectory"]["displacement_world_m"])
        self.basis = projection_basis(displacement, settings["disturbance"]["force_world_n"])
        self.path = 1000*(self.actual-self.desired[0]) @ self.basis.T
        self.goal = 1000*(self.desired-self.desired[0]) @ self.basis.T
        self.length = float(1000*np.linalg.norm(displacement))
        self.ymax_error = max(2., float(np.ceil(max(self.error)/5)*5))
        self.ymax_force = max(1., float(np.ceil(max(self.force))))

    def text(self, draw, xy, label, size=21, fill=INK):
        draw.text(xy, label, font=self.fonts[size], fill=fill)

    def chart(self, draw, box, values, index, ymax, color, times=None):
        left, top, right, bottom = box
        times = self.t[:len(values)] if times is None else times
        duration = float(self.t[-1])
        pulse = self.settings["disturbance"]
        x = lambda t: left+(right-left)*t/duration
        y = lambda v: bottom-(bottom-top)*min(ymax, max(0., v))/ymax
        if pulse["start_s"] < duration:
            draw.rectangle((x(pulse["start_s"]), top, x(min(duration, pulse["start_s"]+pulse["duration_s"])), bottom),
                           fill=(255, 228, 228))
        for f in (0., .5, 1.):
            yy = y(f*ymax)
            draw.line((left, yy, right, yy), fill=(219, 225, 234), width=1)
            self.text(draw, (left-39, yy-10), f"{f*ymax:g}", size=17)
        draw.line((left, bottom, right, bottom), fill=INK, width=1)
        for tick in np.linspace(0, duration, 5):
            self.text(draw, (x(tick)-8, bottom+4), f"{tick:g}", size=17)
        last = min(int(np.searchsorted(times, self.t[index], side="right")), len(values))
        indices = list(range(0, last, 10))
        if last:
            indices.append(last-1)
        points = [(x(times[k]), y(values[k])) for k in indices]
        if len(points) > 1:
            draw.line(points, fill=color, width=3)
        if points:
            xx, yy = points[-1]
            draw.ellipse((xx-5, yy-5, xx+5, yy+5), fill=color)
        draw.line((x(self.t[index]), top, x(self.t[index]), bottom), fill=(100, 116, 139), width=1)

    def compose(self, rgb, index):
        t = float(self.t[index])
        picture = Image.new("RGB", (self.width, self.height), BG)
        picture.paste(Image.fromarray(rgb), (0, 106))
        draw = ImageDraw.Draw(picture)
        draw.rectangle((0, 0, self.width, 105), fill=(15, 28, 48))
        self.text(draw, (24, 10), "沿绿线前进 → 运动中侧推 → 撤力后恢复跟踪", size=32, fill="white")
        pulse = self.settings["disturbance"]
        speed = self.settings.get("presentation", {}).get("playback_speed", .5)
        force_peak = float(max(self.force))
        self.text(draw, (24, 62), f"路程 {self.length/10:.1f} cm   峰值 {force_peak:g} N   推力持续 {pulse['duration_s']:g} s   "
                  f"{self.controller_label}   播放 {speed:g}×   仿真时间 {t:.2f} s", size=24, fill="white")
        caption, color = phase_caption(t, self.settings)
        if force_peak < 1e-8:
            caption, color = "本段记录没有施加外力", GRAY
        if self.status != "completed" and index == len(self.t)-1:
            caption, color = "运行已停止，本次记录没有完成任务", RED
        draw.rounded_rectangle((18, 122, 880, 172), radius=10, fill=color)
        self.text(draw, (34, 129), caption, size=28, fill="white")
        draw.rounded_rectangle((18, 834, 880, 890), radius=8, fill=(15, 28, 48))
        self.text(draw, (30, 842), "蓝色：实际左臂   绿色：目标路径/目标点   红箭头：外力", size=21, fill="white")
        self.text(draw, (930, 119), f"当前施加外力：{self.force[min(index, len(self.force)-1)]:.2f} N", size=28,
                  fill=RED if color == RED else INK)
        self.text(draw, (930, 160), f"末端离当前目标：{self.error[index]:.2f} mm", size=28, fill=BLUE)
        self.text(draw, (930, 205), f"施力时段：{pulse['start_s']:.2f}–{pulse['start_s']+pulse['duration_s']:.2f} 秒", size=21)
        self.text(draw, (930, 252), "末端路径投影：绿线是目标，蓝线是实际", size=21)
        # Equal scale in both projected directions: physical trajectory is not
        # enlarged. The moving green dot is the target at THIS simulation time.
        scale = min(570/(self.length+30), 130/max(30., 2*np.max(np.abs(self.path[:, 1]))+10))
        origin = np.array([966., 374.])
        pixels = lambda positions: origin+positions*np.array([scale, -scale])
        start, end = pixels(np.array([[0., 0.], [self.length, 0.]]))
        draw.line((*start, *end), fill=GREEN, width=4)
        pts = pixels(self.path[:index+1:10])
        if len(pts) > 1:
            draw.line([tuple(point) for point in pts], fill=BLUE, width=3)
        for point, c in ((pixels(self.goal[index]), GREEN), (pixels(self.path[index]), BLUE)):
            draw.ellipse((point[0]-6, point[1]-6, point[0]+6, point[1]+6), fill=c)
        self.text(draw, (start[0]-7, 400), "A", size=21)
        self.text(draw, (end[0]-7, 400), "B", size=21)
        self.text(draw, (930, 447), "路径与偏离使用相同长度比例；机器人位姿未放大。", size=19)
        self.text(draw, (930, 498), "实际施加的推力 (N)", size=24, fill=RED)
        self.chart(draw, (980, 545, 1555, 628), self.force, index, self.ymax_force, RED)
        self.text(draw, (930, 680), "末端位置误差 (mm)", size=24, fill=BLUE)
        self.chart(draw, (980, 733, 1555, 831), self.error, index, self.ymax_error, BLUE)
        if self.nominal is not None:
            # Same nominal controller and reference with zero external force.
            nominal_error = 1000*np.linalg.norm(self.nominal["ee_position_m"]-self.nominal["desired_position_m"], axis=1)
            nt = self.nominal["time_s"]
            last = min(int(np.searchsorted(nt, t)), len(nt)-1)
            points = [(980+575*nt[k]/self.t[-1], 831-98*min(nominal_error[k], self.ymax_error)/self.ymax_error)
                      for k in range(0, last+1, 10)]
            if len(points) > 1:
                draw.line(points, fill=GRAY, width=2)
        self.text(draw, (972, 865), "蓝：有推力   灰：无推力   横轴：仿真时间(s)", size=19)
        return picture


def render_explained_video(output, case, trace, settings, model, data, camera, annotate):
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("Video generation requires system ffmpeg")
    output = Path(output)
    nominal = None
    nominal_path = output / "nominal_tvlqr/trace.npz"
    if nominal_path.exists():
        with np.load(nominal_path) as saved:
            nominal = {key: saved[key] for key in saved.files}
    run_summary = json.loads((output / case / "summary.json").read_text())
    label = {"tvlqr": "时变 LQR", "computed_torque_pd": "计算力矩+PD", "joint_pd": "独立关节PD"}.get(
        run_summary["controller"], run_summary["controller"])
    composition = ExplainedFrame(trace, settings, nominal, controller_label=label, status=run_summary["status"])
    width, height, fps = composition.width, composition.height, 30
    model.vis.global_.offwidth = composition.render_width
    model.vis.global_.offheight = composition.render_height
    speed = settings.get("presentation", {}).get("playback_speed", .5)
    times = trace["time_s"]
    total_frames = int(np.ceil(round(float(times[-1]), 6)/speed*fps))+1
    target = output / f"{case}.mp4"
    command = [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-vcodec", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{width}x{height}", "-r", str(fps), "-i", "-", "-an", "-vcodec", "libx264",
        "-preset", "veryfast", "-pix_fmt", "yuv420p", "-crf", "20", str(target)]
    pulse = settings["disturbance"]
    preview_frame = min(int(round((pulse["start_s"]+.5*pulse["duration_s"])*fps/speed)), total_frames-1)
    recovery_frame = min(int(round((pulse["start_s"]+pulse["duration_s"]+.25)*fps/speed)), total_frames-1)
    with mujoco.Renderer(model, height=composition.render_height, width=composition.render_width) as renderer, \
            subprocess.Popen(command, stdin=subprocess.PIPE) as encoder:
        for k in range(total_frames):
            t = min(times[-1], k*speed/fps)
            index = min(int(np.searchsorted(times, t)), len(times)-1)
            data.qpos[:] = trace["q_rad"][index]
            data.qvel[:] = trace["dq_rad_s"][index]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            annotate(index, renderer.scene)
            picture = composition.compose(renderer.render(), index)
            encoder.stdin.write(np.asarray(picture).tobytes())
            if k == preview_frame:
                picture.save(output / "replay_preview.png")
            if k == recovery_frame:
                picture.save(output / "replay_recovery.png")
            if k and k % 120 == 0:
                print(f"Rendering explained video: {k}/{total_frames} frames", flush=True)
        encoder.stdin.close()
        if encoder.wait() != 0:
            raise RuntimeError("ffmpeg video encoding failed")
    metadata = {"case": case, "width": width, "height": height, "fps": fps, "frames": total_frames,
        "playback_speed": speed, "simulation_duration_s": float(times[-1]),
        "force_start_s": pulse["start_s"], "force_duration_s": pulse["duration_s"],
        "state_geometry": "exact logged poses; equal-scale path projection; no motion exaggeration"}
    (output / f"{case}_video.json").write_text(json.dumps(metadata, indent=2)+"\n")
    print(f"Video: {target}", flush=True)
