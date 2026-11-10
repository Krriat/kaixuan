#!/usr/bin/env python3
"""Real Orin CST test: read-only preflight unless --execute is explicitly supplied."""
import argparse
from contextlib import contextmanager
import csv
from datetime import datetime, timezone
import gc
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from cst_cartesian_math import (JOINTS, RobotModel, TorqueFeedback, hardware_settings,
                                joint_reference, make_plan, rotation_error, validate_resume_pose, vector)
from cst_csv_log import AsyncCsvLog
from cst_lqr_native import config_hash, decode_trace, encode_setpoint


def save_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")


@contextmanager
def execution_scheduling(config):
    """Prioritize only the calling execution thread after all planning/ROS init."""
    settings = config.get("execution_thread")
    if settings is None:
        yield
        return
    cpu, priority = settings["cpu"], settings["fifo_priority"]
    if (not isinstance(cpu, int) or cpu not in os.sched_getaffinity(0)
            or not isinstance(priority, int) or not 1 <= priority < 50):
        raise ValueError("Execution CPU must be available and FIFO priority must be 1-49")
    old_affinity, old_policy, old_priority = os.sched_getaffinity(0), os.sched_getscheduler(0), os.sched_getparam(0)
    try:
        os.sched_setaffinity(0, {cpu})
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
        print(f"Execution thread CPU{cpu}/FIFO{priority}; existing DDS threads unchanged", flush=True)
        yield
    finally:
        os.sched_setscheduler(0, old_policy, old_priority)
        os.sched_setaffinity(0, old_affinity)


class Experiment(Node):
    def __init__(self, config, output, resume_from=None):
        super().__init__("cst_cartesian_z_test")
        self.config, self.output = config, output
        self.native = config.get("runtime_backend", "python") == "ros2_control"
        self.command_topic = "/torque_controller/setpoints" if self.native else "/torque_controller/commands"
        self.native_received = None
        self.resume_from = resume_from
        self.q = self.dq = None
        self.state_effort = np.full(7, np.nan)
        self.stop_reference = None
        self.stop_drift_max = np.zeros(7)
        self.q_received = self.q_stamp = 0.
        self.diagnostics = {}
        self.diagnostics_received = 0.
        self.publisher = None
        self.commanded = False
        self.require_enabled = False
        self.last_tick = None
        self.last_graph_check = 0.
        self.filtered_dq = np.zeros(7)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self.joint_state, qos)
        self.create_subscription(DiagnosticArray, "/kaixuan/hardware_diagnostics", self.diagnostic, qos)
        self.log_file = AsyncCsvLog(output/"samples.csv")
        self.event_log = AsyncCsvLog(output/"events.log", capacity=65536, mirror_stdout=True)
        self.pending_result = None
        self.writer = csv.writer(self.log_file)
        names = ["elapsed_s", "phase", "loop_dt_s", "feedback_age_s", "phase_time_s"]
        if self.native:
            names[3] = "setpoint_age_s"
        for prefix in ("q", "dq", "qr", "dqr", "tau_command", "tau_before_limits", "tau_integral", "tau_measured",
                       "tau_feedforward", "tau_state_feedback", "tau_memory_feedback", "tau_friction_feedforward",
                       "tau_joint_state"):
            names += [f"{prefix}{i+1}" for i in range(7)]
        names += [f"{p}_{a}" for p in ("ee", "ee_reference") for a in "xyz"]
        if self.native:
            names += ["servo_max_period_s", "servo_max_compute_s"]
        self.writer.writerow(names)
        if self.native:
            self.create_subscription(Float64MultiArray, "/torque_controller/servo_trace", self.native_trace, qos)
        self.run_start = time.monotonic()
        self.line_samples = []
        self.stop_after = "line"
        self.last_tick_timing = {}
        self.max_tick_timing = {}
        self.between_tick_timing = {}
        # Keep the node in one executor. rclpy.spin_once(node) otherwise adds and
        # removes it on every call, rebuilding the wait set at control frequency.
        self.io_executor = SingleThreadedExecutor(context=self.context)
        self.io_executor.add_node(self)

    def native_trace(self, message):
        """Record the actual C++ feedback calculation, never a Python shadow command."""
        values, phase = decode_trace(message.data)
        if values[0] < self.run_start:
            return
        self.native_received = time.monotonic()
        q, qr = values[5:12], values[19:26]
        p, rotation, _ = self.model.kinematics(q)
        pr, _, _ = self.model.kinematics(qr)
        self.writer.writerow([values[0]-self.run_start, phase, *values[2:96], *p, *pr, *values[96:]])
        if phase == "line":
            self.line_samples.append((p.copy(), float(np.linalg.norm(rotation_error(self.reference.orientation, rotation))),
                                      float(np.max(np.abs(qr-q)))))

    def spin_once(self, timeout_sec=0.):
        self.io_executor.spin_once(timeout_sec=timeout_sec)

    def destroy_node(self):
        # Caller stops the controller before destruction; blocking drain is safe here.
        self.event_log.close()
        self.io_executor.remove_node(self)
        self.io_executor.shutdown()
        return super().destroy_node()

    def event(self, text):
        # Even a small flushed print to a redirected file/SSH terminal can block.
        # The worker mirrors to stdout and disk without blocking the control thread.
        self.event_log.write(text+"\n")

    def joint_state(self, msg):
        try:
            indices = [msg.name.index(j) for j in JOINTS]
            q = np.array([msg.position[i] for i in indices])
            dq = np.array([msg.velocity[i] for i in indices])
        except (ValueError, IndexError):
            return
        if not np.isfinite(np.r_[q, dq]).all():
            return
        self.q, self.dq = q, dq
        if self.stop_reference is not None:
            self.stop_drift_max = np.maximum(self.stop_drift_max, np.abs(q-self.stop_reference))
        # JointState effort shares the position/velocity acquisition sample.
        # Keep the existing low-rate diagnostic effort column separately.
        self.state_effort = (np.array([msg.effort[i] for i in indices])
                             if len(msg.effort) == len(msg.name) else np.full(7, np.nan))
        self.q_received = time.monotonic()
        self.q_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    def diagnostic(self, msg):
        statuses = {s.name: {v.key: v.value for v in s.values} for s in msg.status if s.name in JOINTS}
        if set(statuses) == set(JOINTS):
            self.diagnostics = statuses
            self.diagnostics_received = time.monotonic()

    def spin(self):
        # Depth-one queues: consume pending state/diagnostics without retaining old samples.
        # There are two depth-one state subscriptions. Extra empty executor
        # polls consume the FIFO thread's CPU budget without improving freshness.
        for _ in range(3 if self.native else 2):
            self.spin_once()

    def service(self, srv_type, name, request, timeout=10.):
        client = self.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Service unavailable: {name}")
            future = client.call_async(request)
            self.io_executor.spin_until_future_complete(future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError(f"Service timeout/error: {name}")
            return future.result()
        finally:
            self.destroy_client(client)

    def parameters(self, node, names):
        request = GetParameters.Request(names=names)
        return self.service(GetParameters, f"/{node}/get_parameters", request).values

    def check_feedback(self):
        now = time.monotonic()
        if self.q is None or now-self.q_received > self.config["max_feedback_age_s"]:
            raise RuntimeError("JointState feedback stale/missing")
        stamp_age = self.get_clock().now().nanoseconds*1e-9-self.q_stamp
        if stamp_age < -.01 or stamp_age > self.config["max_feedback_age_s"]:
            raise RuntimeError(f"JointState acquisition timestamp stale: {stamp_age:.3f}s")
        if now-self.diagnostics_received > self.config["max_diagnostics_age_s"]:
            raise RuntimeError("Hardware diagnostics stale/missing")
        for joint in JOINTS:
            d = self.diagnostics[joint]
            sample_age = now-int(d["sample_steady_ns"])*1e-9
            if sample_age > self.config["max_diagnostics_age_s"]:
                raise RuntimeError(f"{joint}: EtherCAT sample stale: acquisition_age={sample_age:.4f}s, "
                                   f"diagnostic_receipt_age={now-self.diagnostics_received:.4f}s, "
                                   f"joint_state_receipt_age={now-self.q_received:.4f}s")
            if int(d["torque_guard_fault"]) or int(d["error_code"]) or (int(d["status_word"]) & 8):
                raise RuntimeError(f"{joint}: guard={d['torque_guard_fault']}, drive_error={d['error_code']}")
            if int(d["actual_mode"]) != 10 or int(d["effort_claimed"]) != 1:
                raise RuntimeError(f"{joint}: CST mode/effort ownership missing")
            if self.require_enabled and (int(d["status_word"]) & 0x6f) != 0x27:
                raise RuntimeError(f"{joint}: operation-enabled state lost")
        fastest = int(np.argmax(np.abs(self.dq)))
        if abs(self.dq[fastest]) > np.deg2rad(self.config["max_actual_speed_deg_s"]):
            raise RuntimeError(f"{JOINTS[fastest]} experiment speed guard: {np.rad2deg(self.dq[fastest]):.3f} deg/s "
                               f"(limit {self.config['max_actual_speed_deg_s']:.3f})")
        return now

    def preflight(self, prepare_feedback=True):
        description, simulated_time = self.parameters("controller_manager", ["robot_description", "use_sim_time"])
        if simulated_time.bool_value:
            raise RuntimeError("use_sim_time must be false for this real-hardware client")
        xml = description.string_value
        self.hardware_parameters, self.limits = hardware_settings(xml)
        controller = self.service(ListControllers, "/controller_manager/list_controllers", ListControllers.Request())
        controllers = {c.name: c for c in controller.controller}
        bridge = controllers.get("torque_controller")
        expected_type = "kaixuan_can_hardware/LqrTorqueController" if self.native else "kaixuan_can_hardware/TorqueController"
        if bridge is None or bridge.state != "active" or bridge.type != expected_type:
            raise RuntimeError("The TorqueController bridge must be active")
        if set(bridge.claimed_interfaces) != {j+"/effort" for j in JOINTS}:
            raise RuntimeError("Unexpected torque-controller interfaces")
        joints, watchdog = self.parameters("torque_controller", ["joints", "command_timeout"])
        if list(joints.string_array_value) != JOINTS or not 0.02 <= watchdog.double_value <= .1:
            raise RuntimeError("Unexpected joint order or watchdog (must be <= 0.1s)")
        if self.native:
            self.native_shadow = self.parameters("torque_controller", ["shadow_only"])[0].bool_value
            sha, hz, delays, joints, times = self.parameters("torque_controller", [
                "planning_config_sha256", "runtime_hz", "input_delays", "input_joint_indices", "input_time_constants"])
            settings = self.config["lqr"]["input_dynamics"]
            channels = settings.get("channels", [settings])
            if (sha.string_value != config_hash(self.config) or hz.double_value != 1000.
                    or self.config["lqr"].get("runtime_hz") != 1000.
                    or list(delays.integer_array_value) != [c["delay_steps"] for c in channels]
                    or list(joints.integer_array_value) != [JOINTS.index(c["joint"]) for c in channels]
                    or list(times.double_array_value) != [c["time_constant_s"] for c in channels]
                    or self.config["velocity_filter_s"] != 0.
                    or self.config["control_method"] != "tvlqi"):
                raise RuntimeError("Native servo/planner configuration mismatch")
            params = self.parameters("torque_controller", ["effort_limits", "effort_slew", "effort_release_slew",
                                                           "integral_limits", "max_speed_rad_s", "max_tracking_error_rad"])
            expected = [self.limits["effort_limit_nm"], self.limits["effort_slew_nm_s"],
                        self.limits.get("effort_release_slew_nm_s", self.limits["effort_slew_nm_s"]),
                        np.asarray(self.config["integral_limit_nm"])]
            if (any(not np.array_equal(p.double_array_value, v) for p, v in zip(params[:4], expected))
                    or not np.isclose(params[4].double_value, np.deg2rad(self.config["max_actual_speed_deg_s"]), atol=1e-12)
                    or not np.isclose(params[5].double_value, np.deg2rad(self.config["max_tracking_error_deg"]), atol=1e-12)):
                raise RuntimeError("Native servo guard/limit configuration mismatch")
        deadline = time.monotonic()+10.
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=.01)
            if self.q is not None and self.diagnostics:
                break
        self.spin()
        self.check_feedback()
        if any(int(self.diagnostics[j]["torque_armed"]) for j in JOINTS):
            raise RuntimeError("A drive is already armed; do not take over a running torque experiment")
        if self.count_publishers(self.command_topic):
            raise RuntimeError("Another torque-command publisher exists")
        self.initial_q = self.q.copy()
        # This first commissioning test prepares from the observed vertical resting pose.
        resume_info = None
        if self.resume_from is not None:
            source = self.resume_from.resolve()
            snapshot = json.loads(source.read_text(encoding="utf-8"))
            mismatch = validate_resume_pose(snapshot, self.initial_q, self.dq)
            resume_info = {"source": str(source), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                           "mismatch_deg": mismatch.tolist()}
            save_json(self.output/"resume_source.json", snapshot)
        elif np.max(np.abs(self.initial_q)) > np.deg2rad(self.config["max_initial_position_deg"]):
            raise RuntimeError("Initial commissioning requires the left arm near q=0 (vertical rest); see max_initial_position_deg")
        urdf_file = self.output/"active_robot.urdf"
        urdf_file.write_text(xml, encoding="utf-8")
        # Right-arm calibration is explicitly unconfirmed. Its fixed coordinates
        # do not affect the independent fixed-base left-arm dynamics; omit its
        # collision geometry instead of inventing a physical right-arm pose.
        self.model = RobotModel(xml, self.config, np.zeros(7))
        if resume_info is not None:
            gravity = self.model.inverse_dynamics(self.initial_q, np.zeros(7), np.zeros(7))
            if np.any(np.abs(gravity) > .25*self.limits["effort_limit_nm"]):
                raise RuntimeError("Saved-pose restart has insufficient nominal startup gravity margin")
            resume_info["nominal_startup_gravity_nm"] = gravity.tolist()
        print("Planning approach and fixed-orientation world-Z line...", flush=True)
        self.reference, samples, plan = make_plan(self.model, self.initial_q, self.config, self.limits)
        share = Path(get_package_share_directory("kaixuan3_config"))
        model_share = Path(get_package_share_directory("xiaogong_kaixuan"))
        plan.update({"robot_description_sha256": hashlib.sha256(xml.encode()).hexdigest(),
                     "software_sha256": {name: hashlib.sha256(Path(__file__).resolve().with_name(name).read_bytes()).hexdigest()
                         for name in ("cst_cartesian_z.py", "cst_cartesian_math.py", "cst_cartesian_lqr.py",
                                      "cst_cartesian_trajectory.py", "cst_lqr_native.py", "cst_csv_log.py")},
                     "restart_from_saved_stop": resume_info,
                     "right_arm": "uncalibrated; physical pose unknown; excluded from collision checking",
                     "config": self.config,
                     "hardware_limits": {k: v.tolist() for k, v in self.limits.items()},
                     "actuation": "CST effort only; no position/velocity commands"})
        if self.native:
            library = Path(get_package_share_directory("kaixuan_can_hardware")).parent.parent/"lib/libkaixuan_lqr_torque_controller.so"
            plan["native_runtime"] = {"runtime_hz": 1000., "reference_stream_hz": self.config["control_hz"],
                "shadow_only": self.native_shadow, "planning_config_sha256": config_hash(self.config),
                "library_sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
                "log": "C++ servo trace using the current hardware state; Python does not calculate commanded torque"}
        save_json(self.output/"plan.json", plan)
        print("Checking left-arm/body URDF self collisions. Right arm is uncalibrated and excluded.", flush=True)
        try:
            plan["collision_check"] = self.model.check_collisions(urdf_file, [str(model_share.parent)],
                                        share/"config/kaixuan1.srdf", np.vstack((samples[::5], samples[-1])))
        except Exception as error:
            plan["collision_error"] = str(error)
            save_json(self.output/"plan.json", plan)
            raise
        save_json(self.output/"plan.json", plan)
        self.plan = plan
        if not prepare_feedback:
            # Bounded commissioning clients supply their own checked local
            # reference/gain bank before publishing; transport checks above
            # and every runtime guard still apply.
            print("Preflight complete; caller must prepare feedback before execution", flush=True)
            return plan
        method = self.config.get("control_method", "pid")
        if method == "tvlqi":
            from cst_cartesian_lqr import LQRTorqueFeedback
            self.feedback = LQRTorqueFeedback(self.model, self.config, self.limits)
            self.feedback.prepare(self.initial_q, self.reference)
            design = {name: schedule.diagnostics for name, schedule in self.feedback.schedules.items()}
            save_json(self.output/"lqr_design.json", design)
            np.savez_compressed(self.output/"lqr_gains.npz", **{
                name: self.feedback.schedules[name].gains for name in
                ("initial", "target", "final", "approach", "line", "return_line", "return_initial")})
        elif method == "pid":
            self.feedback = TorqueFeedback(self.model, self.config, self.limits)
        else:
            raise ValueError(f"Unknown control_method: {method}")
        print(f"Prepared feedback: {method}; no torque published", flush=True)
        print(json.dumps({k: plan[k] for k in ("initial_joints_deg", "target_joints_deg", "final_joints_deg",
                                             "start_ee_world_m", "end_ee_world_m", "collision_check")}, indent=2), flush=True)
        return plan

    def tick(self, reference, phase, scale=1., zero=False, phase_time=0.):
        tick_enter = time.monotonic()
        cpu_enter = time.thread_time()
        self.spin()
        after_spin = time.monotonic()
        now = self.check_feedback()
        dt = 1./self.config["control_hz"] if self.last_tick is None else now-self.last_tick
        if dt > self.config["max_loop_gap_s"] or dt <= 0.:
            raise RuntimeError(f"Control loop stalled: {dt:.4f}s; current spin={after_spin-tick_enter:.4f}s; "
                               f"previous tick sections={self.last_tick_timing}; "
                               f"between ticks={self.between_tick_timing}")
        self.last_tick = now
        if now-self.last_graph_check > 1.:
            if self.count_publishers(self.command_topic) != 1:
                raise RuntimeError("Torque command topic must have exactly this one publisher")
            self.last_graph_check = now
        after_graph = time.monotonic()
        qr, dqr, _ = reference
        if np.max(np.abs(qr-self.q)) > np.deg2rad(self.config["max_tracking_error_deg"]):
            raise RuntimeError(f"Tracking error guard: {np.rad2deg(qr-self.q).round(3).tolist()} deg")
        if np.any(self.q <= self.limits["min_position_rad"]) or np.any(self.q >= self.limits["max_position_rad"]):
            raise RuntimeError("Actual position outside hardware bounds")
        if np.any(np.abs(self.q-self.initial_q) >= self.limits["max_displacement_rad"]):
            raise RuntimeError("Actual position outside displacement guard")
        if self.native:
            if self.require_enabled and (self.native_received is None or now-self.native_received > .05):
                raise RuntimeError("Native servo trace missing/stale")
            data = encode_setpoint(phase, time.monotonic(), phase_time, scale, reference, self.feedback, zero)
            after_calculation = time.monotonic()
            self.commanded = True
            self.publisher.publish(Float64MultiArray(data=data))
            end = time.monotonic()
            self.last_tick_timing = {"spin_s": after_spin-tick_enter, "checks_graph_s": after_graph-after_spin,
                "calculation_s": after_calculation-after_graph, "publish_s": end-after_calculation,
                "total_wall_s": end-tick_enter, "total_cpu_s": time.thread_time()-cpu_enter}
            for key, value in self.last_tick_timing.items():
                self.max_tick_timing[key] = max(self.max_tick_timing.get(key, 0.), value)
            return
        alpha = dt/(self.config["velocity_filter_s"]+dt)
        self.filtered_dq += alpha*(self.dq-self.filtered_dq)
        if zero:
            command, raw, integral = np.zeros(7), np.zeros(7), np.zeros(7)
        else:
            self.feedback.select(phase, phase_time)
            command, raw, integral = self.feedback.step(self.q, self.filtered_dq, reference, dt, scale)
        if not np.isfinite(np.r_[command, raw]).all():
            raise RuntimeError("Nonfinite torque calculation")
        after_calculation = time.monotonic()
        self.commanded = True
        self.publisher.publish(Float64MultiArray(data=command.tolist()))
        after_publish = time.monotonic()
        p, rotation, _ = self.model.kinematics(self.q)
        pr, _, _ = self.model.kinematics(qr)
        measured = [float(self.diagnostics[j]["effort_nm"]) for j in JOINTS]
        feedforward = np.zeros(7) if zero else self.feedback.feedforward
        state_feedback = np.zeros(7) if zero else self.feedback.state_feedback
        memory_feedback = np.zeros(7) if zero else getattr(self.feedback, "memory_feedback", np.zeros(7))
        friction_feedforward = np.zeros(7) if zero else getattr(self.feedback, "friction_feedforward", np.zeros(7))
        self.writer.writerow([now-self.run_start, phase, dt, now-self.q_received, phase_time,
                              *self.q, *self.dq, *qr, *dqr, *command, *raw, *integral, *measured,
                              *feedforward, *state_feedback, *memory_feedback, *friction_feedforward,
                              *self.state_effort, *p, *pr])
        if phase == "line":
            self.line_samples.append((p.copy(), float(np.linalg.norm(rotation_error(self.reference.orientation, rotation))),
                                      float(np.max(np.abs(qr-self.q)))))
        tick_end = time.monotonic()
        self.last_tick_timing = {"spin_s": after_spin-tick_enter, "checks_graph_s": after_graph-after_spin,
            "calculation_s": after_calculation-after_graph, "publish_s": after_publish-after_calculation,
            "fk_log_s": tick_end-after_publish, "total_wall_s": tick_end-tick_enter,
            "total_cpu_s": time.thread_time()-cpu_enter}
        for key, value in self.last_tick_timing.items():
            self.max_tick_timing[key] = max(self.max_tick_timing.get(key, 0.), value)

    def run_phase(self, name, duration, function, scale_function=None):
        self.event(f"PHASE {name}: {duration:.1f}s")
        begin = time.monotonic()
        period = 1./self.config["control_hz"]
        while True:
            elapsed = min(time.monotonic()-begin, duration)
            reference_begin = time.monotonic()
            reference = function(elapsed)
            self.between_tick_timing["reference_s"] = time.monotonic()-reference_begin
            self.tick(reference, name, scale_function(elapsed) if scale_function else 1., phase_time=elapsed)
            if elapsed >= duration:
                break
            sleep_begin = time.monotonic()
            requested = max(0., begin+elapsed+period-sleep_begin)
            time.sleep(requested)
            self.between_tick_timing.update(sleep_requested_s=requested,
                                           sleep_actual_s=time.monotonic()-sleep_begin)

    def settle(self, target, name):
        self.event(f"PHASE {name}: waiting for position/velocity tolerance")
        start = time.monotonic()
        stable_since = None
        while time.monotonic()-start < self.config["settle_timeout_s"]:
            tick_start = time.monotonic()
            self.tick((target, np.zeros(7), np.zeros(7)), name, phase_time=tick_start-start)
            good = (np.max(np.abs(self.q-target)) < np.deg2rad(self.config["settle_position_deg"])
                    and np.max(np.abs(self.dq)) < np.deg2rad(self.config["settle_velocity_deg_s"]))
            stable_since = (stable_since or tick_start) if good else None
            if stable_since is not None and tick_start-stable_since >= self.config["settle_duration_s"]:
                return
            time.sleep(max(0., tick_start+1./self.config["control_hz"]-time.monotonic()))
        raise RuntimeError(f"{name}: did not settle; error deg={np.rad2deg(target-self.q).round(3).tolist()}")

    def result(self):
        end, rotation, _ = self.model.kinematics(self.q)
        positions = np.array([s[0] for s in self.line_samples])
        expected = self.reference.start_position+self.reference.displacement
        direction = self.reference.displacement/np.linalg.norm(self.reference.displacement)
        offset = positions-self.reference.start_position
        lateral = np.linalg.norm(offset-np.outer(offset@direction, direction), axis=1)
        final_error = float(np.linalg.norm(end-expected))
        orientation = max([s[1] for s in self.line_samples] + [float(np.linalg.norm(rotation_error(self.reference.orientation, rotation)))])
        result = {"measurement": "encoder FK using active URDF, not external end-effector metrology",
                  "actual_start_ee_world_m": positions[0].tolist(), "actual_end_ee_world_m": end.tolist(),
                  "actual_displacement_m": (end-positions[0]).tolist(), "target_end_ee_world_m": expected.tolist(),
                  "final_position_error_m": final_error, "max_lateral_error_m": float(max(lateral)),
                  "max_orientation_error_deg": float(np.rad2deg(orientation)),
                  "max_joint_tracking_error_deg": float(np.rad2deg(max(s[2] for s in self.line_samples)))}
        result["accepted"] = bool(final_error <= self.config["acceptance_final_position_m"]
            and max(lateral) <= self.config["acceptance_lateral_error_m"]
            and np.rad2deg(orientation) <= self.config["acceptance_orientation_deg"])
        self.pending_result = result
        self.event("MEASURED RESULT " + json.dumps(result, ensure_ascii=False))
        return result

    def execute(self, finish):
        if self.native and self.native_shadow:
            raise RuntimeError("This controller is configured for shadow computation; physical execution is disabled")
        # Planning/mesh loading is complete before any drive is enabled.
        # Precomputation can take seconds. Drain fresh state before checking it.
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=.01)
            self.spin()
            now = time.monotonic()
            if self.q is not None and now-self.q_received < .01 and now-self.diagnostics_received < .3:
                break
        self.check_feedback()
        if np.max(np.abs(self.q-self.initial_q)) > np.deg2rad(.25):
            raise RuntimeError("Arm moved during planning; rerun preflight")
        if any(int(self.diagnostics[j]["torque_armed"]) for j in JOINTS):
            raise RuntimeError("Arm was armed during planning")
        self.filtered_dq = self.dq.copy()
        self.publisher = self.create_publisher(Float64MultiArray, self.command_topic, 1)
        deadline = time.monotonic()+3.
        while self.publisher.get_subscription_count() != 1 and time.monotonic() < deadline:
            self.spin_once(timeout_sec=.01)
        if self.publisher.get_subscription_count() != 1:
            raise RuntimeError("Expected one torque bridge subscriber")
        print("EXECUTING ON REAL LEFT ARM: zero handshake, enable, gravity ramp, approach, +Z line", flush=True)
        self.run_start = time.monotonic()
        hold_initial = lambda t: (self.initial_q, np.zeros(7), np.zeros(7))
        while time.monotonic()-self.run_start < 3.:
            begin = time.monotonic()
            self.tick(hold_initial(0), "enable_zero", zero=True)
            if np.max(np.abs(self.q-self.initial_q)) > np.deg2rad(2.):
                raise RuntimeError("Excess drift during zero-torque enable")
            if all((int(self.diagnostics[j]["status_word"]) & 0x6f) == 0x27 for j in JOINTS):
                break
            time.sleep(max(0., begin+1./self.config["control_hz"]-time.monotonic()))
        else:
            raise RuntimeError("Not all seven drives enabled within 3s")
        self.require_enabled = True
        ramp = self.config["gravity_ramp_s"]
        self.run_phase("ramp", ramp, hold_initial, lambda t: t/ramp)
        self.settle(self.initial_q, "settle_initial")
        if self.stop_after == "hold":
            self.run_phase("commission_hold", 10., hold_initial)
            self.settle(self.initial_q, "settle_initial")
            result = {"accepted": True, "scope": "initial-pose hold only; no Cartesian-path acceptance",
                      "control_method": self.config.get("control_method", "pid"),
                      "joint_error_deg": np.rad2deg(self.initial_q-self.q).tolist()}
            self.pending_result = result
            self.run_phase("ramp_down_at_rest", ramp, hold_initial, lambda t: 1.-t/ramp)
            return result
        target = np.deg2rad(vector(self.config, "target_joints_deg"))
        self.run_phase("approach", self.config["approach_s"],
                       lambda t: joint_reference(self.initial_q, target, t, self.config["approach_s"]))
        self.settle(target, "settle_target")
        if self.stop_after == "approach":
            result = {"accepted": True, "scope": "approach and settle only; no Cartesian-path acceptance",
                      "control_method": self.config.get("control_method", "pid"),
                      "joint_error_deg": np.rad2deg(target-self.q).tolist()}
            self.pending_result = result
            self.run_phase("return_initial", self.config["approach_s"],
                           lambda t: joint_reference(target, self.initial_q, t, self.config["approach_s"]))
            self.settle(self.initial_q, "settle_return_initial")
            self.run_phase("ramp_down_at_rest", ramp, hold_initial, lambda t: 1.-t/ramp)
            return result
        self.run_phase("line", self.reference.times[-1], self.reference.sample)
        self.settle(self.reference.q[-1], "settle_final")
        hold_final = lambda t: (self.reference.q[-1], np.zeros(7), np.zeros(7))
        self.run_phase("final_hold", self.config["final_hold_s"], hold_final)
        result = self.result()
        if finish == "return":
            def backwards(t):
                q, dq, ddq = self.reference.sample(self.reference.times[-1]-t)
                return q, -dq, ddq
            self.run_phase("return_line", self.reference.times[-1], backwards)
            self.settle(target, "settle_return_target")
            self.run_phase("return_initial", self.config["approach_s"],
                           lambda t: joint_reference(target, self.initial_q, t, self.config["approach_s"]))
            self.settle(self.initial_q, "settle_return_initial")
            self.run_phase("ramp_down_at_rest", ramp, hold_initial, lambda t: 1.-t/ramp)
        else:
            self.event("Holding final pose with CST. Ctrl+C invalidates torque and disables; support the arm before stopping.")
            while True:
                self.run_phase("hold", 5., hold_final)
                self.log_file.flush()
        return result

    def stop(self):
        if not self.commanded:
            return
        try:
            self.event("STOP: invalidate all torque commands and deactivate torque_controller.")
        except Exception:
            pass  # A failed logger must never prevent command invalidation.
        self.stop_reference = self.q.copy()
        self.stop_drift_max = np.zeros(7)
        for _ in range(4):
            self.publisher.publish(Float64MultiArray(data=[float("nan")]*7))
            self.spin_once(timeout_sec=.02)
        request = SwitchController.Request(deactivate_controllers=["torque_controller"], strictness=2)
        request.timeout.sec = 2
        response = self.service(SwitchController, "/controller_manager/switch_controller", request, timeout=3.)
        if not response.ok:
            raise RuntimeError("Controller deactivation failed; command watchdog remains the stop fallback")
        # Capture final drive states without claiming that CW6 guarantees mechanical braking.
        deadline = time.monotonic()+.5
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=.01)
        save_json(self.output/"stop_state.json", {"diagnostics": self.diagnostics,
                  "joints_deg": np.rad2deg(self.q).tolist(), "velocity_deg_s": np.rad2deg(self.dq).tolist(),
                  "pre_stop_joints_deg": np.rad2deg(self.stop_reference).tolist(),
                  "max_observed_stop_drift_deg": np.rad2deg(self.stop_drift_max).tolist()})


def main():
    share = Path(get_package_share_directory("kaixuan3_config"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Enable and physically move the real left arm")
    parser.add_argument("--finish", choices=("hold", "return"), default="hold",
                        help="hold the final pose until Ctrl+C, or reverse the tested path and disable at initial rest")
    parser.add_argument("--config", type=Path, default=share/"config/cst_cartesian_lqr.yaml")
    parser.add_argument("--stop-after", choices=("hold", "approach", "line"), default="line",
                        help="Commission initial hold or approach/return before attempting the complete Cartesian path")
    parser.add_argument("--resume-from", type=Path,
                        help="Restart from final_standby.json matching the current disabled pose within 0.25 degrees")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    for key, value in config.items():
        if key == "velocity_filter_s":
            if not isinstance(value, (float, int)) or not np.isfinite(value) or value < 0.:
                raise ValueError("velocity_filter_s must be finite and nonnegative; zero disables filtering")
            continue
        if isinstance(value, (float, int)) and (not np.isfinite(value) or value <= 0):
            raise ValueError(f"{key} must be finite and positive")
    if not 100. <= config["control_hz"] <= 500. or min(config["approach_s"], config["line_s"]) < 10.:
        raise ValueError("Use 100-500 Hz and at least 10s motion phases for this commissioning test")
    output = args.output or Path.cwd()/"log"/("cst_z_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ"))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    print(f"Artifacts: {output}", flush=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Experiment(config, output, args.resume_from)
    node.stop_after = args.stop_after
    status, code = {"executed": False}, 0
    gc_was_enabled = gc.isenabled()
    try:
        node.preflight()
        if args.execute:
            status["executed"] = True
            # Refcounting remains active. Only defer cyclic collection for a
            # bounded commissioning run; indefinite hold must keep collecting.
            gc.collect()
            status["cyclic_gc_deferred"] = args.finish == "return" or args.stop_after != "line"
            if status["cyclic_gc_deferred"]:
                gc.disable()
            with execution_scheduling(config):
                result = node.execute(args.finish)
            status["status"] = "completed" if result["accepted"] else "completed_outside_tracking_tolerance"
            code = 0 if result["accepted"] else 2
        else:
            status["status"] = "preflight_passed_no_torque_published"
            print("PREFLIGHT PASSED. No torque published, no drive enabled. Add --execute to move.", flush=True)
    except KeyboardInterrupt:
        status.update(status="interrupted", reason="Ctrl+C")
        code = 130
    except Exception as error:
        status.update(status="aborted", reason=str(error))
        if node.q is not None:
            status["abort_joints_deg"] = np.rad2deg(node.q).tolist()
            status["abort_velocity_deg_s"] = np.rad2deg(node.dq).tolist()
            status["abort_diagnostics"] = node.diagnostics
        # Do not let logging delay finally's command invalidation.
        status["abort_notice"] = f"ABORT: {error}"
        code = 1
    finally:
        try:
            node.stop()
        except Exception as error:
            status["stop_error"] = str(error)
            print(f"STOP ERROR: {error}", flush=True)
            code = 1
        status["torque_was_published"] = node.commanded
        if "abort_notice" in status:
            print(status.pop("abort_notice"), flush=True)
        status["maximum_tick_sections_s"] = node.max_tick_timing
        status["csv_backlog_high_water_bytes"] = node.log_file.high_water_bytes
        if node.pending_result is not None:
            save_json(output/"result.json", node.pending_result)
        try:
            node.log_file.close()
        except Exception as error:
            status["csv_close_error"] = str(error)
            code = 1
        save_json(output/"status.json", status)
        if gc_was_enabled:
            gc.enable()
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
