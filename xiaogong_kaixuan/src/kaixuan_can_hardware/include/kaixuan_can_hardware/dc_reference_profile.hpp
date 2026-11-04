#pragma once

#include "kaixuan_can_hardware/dc_reference_follower.hpp"

namespace kaixuan_can_hardware
{
// Fixed optional reference mode profile. Host mode retains the helper defaults.
// Shared by the hardware adapter and regression tests to prevent gain drift.
inline DcReferenceFollower::Config reference_follow_config()
{
  DcReferenceFollower::Config config;
  config.proportional_gain = 0.03;
  config.integral_gain = 0.000075;
  config.fine_entry_filtered_ns = 60000;
  config.bumpless_fine_entry = true;
  return config;
}
}  // namespace kaixuan_can_hardware
