#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace kaixuan_can_hardware
{
constexpr double kJointAnglePi = 3.14159265358979323846;

inline double normalize_joint_angle(double radians)
{
  double value = std::fmod(radians + kJointAnglePi, 2.0 * kJointAnglePi);
  if (value < 0.0) {value += 2.0 * kJointAnglePi;}
  return value - kJointAnglePi;
}

struct JointAngleMapping
{
  double gear_ratio, offset_deg, direction;
  bool normalized{false};

  double unwrapped_position(std::int32_t counts) const
  {
    return direction * (counts * 360.0 / (1048576.0 * gear_ratio) - offset_deg) *
      kJointAnglePi / 180.0;
  }

  double position(std::int32_t counts) const
  {
    const double radians = unwrapped_position(counts);
    return normalized ? normalize_joint_angle(radians) : radians;
  }

  std::int32_t target_counts(double radians, std::int32_t actual_counts) const
  {
    const double counts_per_turn = 1048576.0 * gear_ratio;
    double target = (direction * radians * 180.0 / kJointAnglePi + offset_deg) *
      counts_per_turn / 360.0;
    if (normalized) {
      // Equivalent angles must hold the current encoder turn, not move a full turn.
      target += std::round((actual_counts - target) / counts_per_turn) * counts_per_turn;
    }
    target = std::clamp(target,
      static_cast<double>(std::numeric_limits<std::int32_t>::min()),
      static_cast<double>(std::numeric_limits<std::int32_t>::max()));
    return static_cast<std::int32_t>(std::llround(target));
  }
};
}  // namespace kaixuan_can_hardware
