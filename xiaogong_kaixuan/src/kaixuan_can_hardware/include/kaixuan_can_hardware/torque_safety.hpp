#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace kaixuan_can_hardware
{
// ROS effort is output-joint N.m. The command scale is firmware-specific;
// feedback scaling is selected from the drive's Pn083 and Pn085 readings.
struct TorqueParameters
{
  double rated_nm{0.0};
  double command_units_per_nm{0.0};
  double limit_nm{0.0};
  double slew_nm_s{0.0};
  double max_velocity_rad_s{0.0};
  double max_displacement_rad{0.0};
  double min_position_rad{-3.14}, max_position_rad{3.14};
  bool normalize_position{false};

  bool valid() const
  {
    for (double v : {rated_nm, command_units_per_nm, limit_nm, slew_nm_s,
        max_velocity_rad_s, max_displacement_rad})
    {
      if (!std::isfinite(v) || v <= 0.0) {return false;}
    }
    return std::isfinite(min_position_rad) && std::isfinite(max_position_rad) &&
      min_position_rad < max_position_rad && limit_nm <= rated_nm &&
      limit_nm * command_units_per_nm >= 1.0 &&
      limit_nm * command_units_per_nm <= 32767.0;
  }

  std::int16_t to_raw(double effort, double direction) const
  {
    const auto bound = static_cast<long>(std::floor(limit_nm * command_units_per_nm));
    const double limited = std::clamp(effort, -limit_nm, limit_nm);
    return static_cast<std::int16_t>(std::clamp(
      std::lround(direction * limited * command_units_per_nm), -bound, bound));
  }

  double feedback_nm(std::int16_t raw, double direction,
    unsigned pn083, double rated_current_a) const
  {
    const double scale = pn083 == 0 ? rated_nm / 1000.0 :
      rated_nm * 0.01 / rated_current_a;
    return direction * raw * scale;
  }
};

struct TorqueSafety
{
  enum class Fault : unsigned
  {
    None, InvalidCommand, WriteTimeout, Communication, Mode,
    Drive, Position, Velocity, Displacement, Feedback, Peer
  };
  struct Input
  {
    bool requested{false}, startup_ready{false}, bus_ready{false}, write_fresh{false};
    int mode{0};
    std::uint16_t status{0}, error{0};
    double command{std::numeric_limits<double>::quiet_NaN()};
    double position{0.0}, velocity{0.0}, period_s{0.001};
    double unwrapped_position{0.0};
  };
  struct Output {bool enable{false}; double effort_nm{0.0};};
  struct FaultSnapshot
  {
    bool valid{false};
    Input input{};
    double anchor{0.0}, displacement{0.0};
  };
  Fault fault{Fault::None};
  FaultSnapshot fault_snapshot{};
  bool armed{false}, ever_enabled{false};
  double anchor{0.0}, output_nm{0.0};

  // Reset only on a controller claim/release or hardware lifecycle transition.
  void reset() {*this = TorqueSafety{};}
  void trip(Fault reason) {if (fault == Fault::None) {fault = reason;} output_nm = 0.0;}
  void capture_fault(const Input & in, double displacement_position)
  {
    if (fault_snapshot.valid) {return;}
    fault_snapshot.valid = true;
    fault_snapshot.input = in;
    fault_snapshot.anchor = anchor;
    fault_snapshot.displacement = displacement_position - anchor;
  }
  static const char * fault_name(Fault reason)
  {
    switch (reason) {
      case Fault::None: return "none";
      case Fault::InvalidCommand: return "invalid_or_timed_out_command";
      case Fault::WriteTimeout: return "hardware_write_timeout";
      case Fault::Communication: return "bus_or_dc_not_ready";
      case Fault::Mode: return "mode_mismatch";
      case Fault::Drive: return "drive_fault_or_unexpected_disable";
      case Fault::Position: return "absolute_position_limit";
      case Fault::Velocity: return "velocity_limit";
      case Fault::Displacement: return "relative_displacement_limit";
      case Fault::Feedback: return "invalid_feedback";
      case Fault::Peer: return "peer_fault";
    }
    return "unknown";
  }

  Output step(const Input & in, const TorqueParameters & p)
  {
    const double displacement_position = p.normalize_position ? in.unwrapped_position : in.position;
    if (!in.requested) {output_nm = 0.0; return {};}
    if (fault != Fault::None) {return {};}
    if (!armed) {
      // Starting a controller, or a stale nonzero command, cannot release a brake.
      if (!in.startup_ready || !in.bus_ready || !in.write_fresh || in.mode != 10 ||
        !std::isfinite(in.command) || std::abs(in.command) > 1e-9)
      {return {};}
      anchor = displacement_position;
      armed = true;
    }
    if (!std::isfinite(in.command)) {trip(Fault::InvalidCommand);}
    else if (!in.write_fresh) {trip(Fault::WriteTimeout);}
    else if (!in.startup_ready || !in.bus_ready) {trip(Fault::Communication);}
    else if (in.mode != 10) {trip(Fault::Mode);}
    else if (!std::isfinite(in.position) || !std::isfinite(displacement_position) ||
      !std::isfinite(in.velocity) ||
      !std::isfinite(in.period_s) || in.period_s <= 0.0 || in.period_s > 0.1)
    {trip(Fault::Feedback);}
    else if (in.position < p.min_position_rad || in.position > p.max_position_rad)
    {trip(Fault::Position);}
    else if (std::abs(in.velocity) > p.max_velocity_rad_s) {trip(Fault::Velocity);}
    else if (std::abs(displacement_position - anchor) > p.max_displacement_rad) {trip(Fault::Displacement);}
    else if (((in.status & 8U) || in.error != 0) &&
      (ever_enabled || (in.error != 0 && in.error != 300 && in.error != 400)))
    {trip(Fault::Drive);}
    if (fault != Fault::None) {capture_fault(in, displacement_position); return {};}
    if ((in.status & 0x006fU) != 0x0027U) {
      if (ever_enabled) {trip(Fault::Drive); capture_fault(in, displacement_position); return {};}
      output_nm = 0.0;
      return {true, 0.0};
    }
    ever_enabled = true;
    const double desired = std::clamp(in.command, -p.limit_nm, p.limit_nm);
    const double step = p.slew_nm_s * in.period_s;
    output_nm += std::clamp(desired - output_nm, -step, step);
    return {true, output_nm};
  }
};

template<std::size_t N>
bool latch_torque_group(std::array<TorqueSafety, N> & guards, const std::array<bool, N> & claimed)
{
  bool fault = false;
  for (std::size_t i = 0; i < N; ++i) {
    fault = fault || (claimed[i] && guards[i].fault != TorqueSafety::Fault::None);
  }
  if (fault) {
    for (std::size_t i = 0; i < N; ++i) {
      if (claimed[i]) {guards[i].trip(TorqueSafety::Fault::Peer);}
    }
  }
  return fault;
}
}  // namespace kaixuan_can_hardware
