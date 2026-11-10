"""Fixed protocol for the optional ros2_control LQI runtime (no ROS imports)."""
import hashlib
import json

import numpy as np

PHASES = ("enable_zero", "ramp", "settle_initial", "commission_hold", "approach",
          "settle_target", "line", "settle_final", "final_hold", "return_line",
          "settle_return_target", "return_initial", "settle_return_initial",
          "ramp_down_at_rest", "hold")


def config_hash(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def encode_setpoint(phase, sent_s, elapsed, scale, reference, feedback, zero=False):
    qr, vr, ar = reference
    states = 21+len(feedback.input_model.state)
    if zero:
        ff, friction, gain, inverse = np.zeros(7), np.zeros(7), np.zeros((7, states)), np.zeros((7, 7))
    else:
        feedback.select(phase, elapsed)
        friction = feedback.friction*np.tanh(vr/feedback.friction_speed)
        ff = feedback.model.inverse_dynamics(qr, vr, ar)+friction
        gain, inverse = feedback.gain, feedback.inverse_integral_gain
    if gain.shape != (7, states) or inverse.shape != (7, 7):
        raise ValueError("Native LQI gain shape mismatch")
    data = np.r_[1., 1. if zero else 2., PHASES.index(phase), sent_s, scale, elapsed,
                 qr, vr, ff, friction, gain.ravel(), inverse.ravel()]
    if not np.isfinite(data).all() or not 0 <= scale <= 1 or elapsed < 0:
        raise ValueError("Invalid native LQI setpoint")
    return data.tolist()


def decode_trace(data):
    a = np.asarray(data, dtype=float)
    if a.shape != (98,) or not np.isfinite(a).all() or a[1] != int(a[1]) or not 0 <= a[1] < len(PHASES):
        raise ValueError("Invalid native servo trace")
    return a, PHASES[int(a[1])]
