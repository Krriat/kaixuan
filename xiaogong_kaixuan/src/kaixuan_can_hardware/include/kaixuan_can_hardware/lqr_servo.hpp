#pragma once

// Fixed-size counterpart of cst_cartesian_lqr.LQRTorqueFeedback. Planning,
// Pinocchio and Riccati recursion stay outside the servo thread.
#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include "kaixuan_can_hardware/torque_safety.hpp"

namespace kaixuan_can_hardware
{
struct LqrSetpoint
{
  static constexpr std::size_t N = 7, MAX_STATE = 32;
  using Vec = std::array<double, N>;
  int mode{0}, phase{-1};  // 0 invalid; 1 explicit zero handshake; 2 feedback.
  double sent_s{0}, received_s{0}, scale{0}, phase_time_s{0};
  Vec q{}, v{}, feedforward{}, friction{};
  std::array<double, N * MAX_STATE> gain{};
  std::array<double, N * N> inverse_integral{};
  bool valid{false};
};

struct LqrServo
{
  using Vec = LqrSetpoint::Vec;
  Vec limit{}, increase{}, release{}, integral_limit{};
  unsigned delay{3}, input_joint{3};
  double time_constant{0.0034};
  struct Channel {unsigned joint{3}, delay{3}; double time_constant{.0034};};
  std::array<Channel, 7> channels{};
  unsigned channel_count{0};  // Zero preserves the original single-channel settings.
  Vec previous{}, integral{}, z{}, raw{}, state_feedback{}, memory_feedback{};
  std::array<double, 11> input_state{};
  int phase{-1};

  std::size_t states() const
  {
    std::size_t result = 21;
    if (!channel_count) {return 22+delay;}
    for (unsigned i = 0; i < channel_count && i < channels.size(); ++i) {result += 1+channels[i].delay;}
    return result;
  }
  void reset()
  {
    previous.fill(0); integral.fill(0); z.fill(0); raw.fill(0);
    state_feedback.fill(0); memory_feedback.fill(0); input_state.fill(0); phase = -1;
  }
  void restore_integral_coordinates(const LqrSetpoint & s)
  {
    for (std::size_t i = 0; i < 7; ++i) {
      z[i] = 0;
      for (std::size_t j = 0; j < 7; ++j) {z[i] -= s.inverse_integral[i*7+j]*integral[j];}
    }
  }
  double bound(std::size_t i, double desired, double dt) const
  {
    return slew_torque(previous[i], std::clamp(desired, -limit[i], limit[i]),
      dt, increase[i], release[i]);
  }
  bool step(const LqrSetpoint & s, const Vec & q, const Vec & v, double dt)
  {
    if (!s.valid || s.mode < 1 || s.mode > 2 || !std::isfinite(dt) || dt <= 0 || dt > .01 ||
      channel_count > channels.size() || states() > LqrSetpoint::MAX_STATE ||
      !std::isfinite(s.scale) || s.scale < 0 || s.scale > 1) {return false;}
    if (s.mode == 1) {reset(); return true;}
    if (s.phase != phase) {restore_integral_coordinates(s); phase = s.phase;}
    std::array<double, 11> nominal{};
    std::size_t offset = 0;
    for (unsigned k = 0; k < (channel_count ? channel_count : 1); ++k) {
      const Channel c = channel_count ? channels[k] : Channel{input_joint, delay, time_constant};
      if (c.joint >= 7 || c.delay > 10 || !std::isfinite(c.time_constant) || c.time_constant <= 0) {return false;}
      const double delayed = c.delay ? input_state[offset+1] : previous[c.joint];
      const double rho = std::exp(-dt/c.time_constant);
      input_state[offset] = rho*input_state[offset] + (1-rho)*delayed;
      if (c.delay) {
        for (std::size_t i = 1; i < c.delay; ++i) {input_state[offset+i] = input_state[offset+i+1];}
        input_state[offset+c.delay] = previous[c.joint];
      }
      for (unsigned i = 0; i <= c.delay; ++i) {nominal[offset+i] = s.scale*s.feedforward[c.joint];}
      offset += 1+c.delay;
    }
    Vec candidate_z = z, old_i{}, candidate_i{}, next{};
    for (std::size_t j = 0; j < 7; ++j) {
      if (!std::isfinite(q[j]) || !std::isfinite(v[j])) {return false;}
      if (s.scale >= 1) {candidate_z[j] += (q[j]-s.q[j])*dt;}
    }
    for (std::size_t i = 0; i < 7; ++i) {
      const auto offset = i*LqrSetpoint::MAX_STATE;
      state_feedback[i] = memory_feedback[i] = 0;
      for (std::size_t j = 0; j < 7; ++j) {
        state_feedback[i] -= s.gain[offset+j]*(q[j]-s.q[j]) + s.gain[offset+7+j]*(v[j]-s.v[j]);
        old_i[i] -= s.gain[offset+14+j]*z[j];
        candidate_i[i] -= s.gain[offset+14+j]*candidate_z[j];
      }
      for (std::size_t j = 0; j < states()-21; ++j) {
        memory_feedback[i] -= s.scale*s.gain[offset+21+j]*(input_state[j]-nominal[j]);
      }
      old_i[i] = std::clamp(old_i[i], -integral_limit[i], integral_limit[i]);
      candidate_i[i] = std::clamp(candidate_i[i], -integral_limit[i], integral_limit[i]);
      raw[i] = s.scale*(s.feedforward[i]+state_feedback[i]+candidate_i[i])+memory_feedback[i];
      if (!std::isfinite(raw[i])) {return false;}
      const double command = bound(i, raw[i], dt);
      const double delta_i = candidate_i[i]-old_i[i];
      const bool outward = (raw[i]-command > 1e-10 && delta_i > 0) ||
        (raw[i]-command < -1e-10 && delta_i < 0);
      integral[i] = outward ? old_i[i] : candidate_i[i];
      raw[i] = s.scale*(s.feedforward[i]+state_feedback[i]+integral[i])+memory_feedback[i];
      next[i] = bound(i, raw[i], dt);
      if (!std::isfinite(next[i])) {return false;}
    }
    restore_integral_coordinates(s);
    previous = next;
    return true;
  }
};
}  // namespace kaixuan_can_hardware
