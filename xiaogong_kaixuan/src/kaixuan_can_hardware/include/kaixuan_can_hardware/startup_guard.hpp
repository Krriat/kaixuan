#pragma once

#include <cstdint>
#include <limits>

namespace kaixuan_can_hardware
{
// Startup qualification is latched once enabled. A single subsequent bus
// disturbance must not unexpectedly remove holding torque from an arm.
struct StartupGuard
{
  static constexpr unsigned kBusCycles = 100;
  static constexpr unsigned kDcSamples = 5;
  static constexpr unsigned kDcMaxAgeCycles = 100;
  static constexpr std::uint32_t kDcLimitNs = 50000;
  unsigned bus_cycles{0}, dc_good_samples{0}, dc_age{kDcMaxAgeCycles + 1};
  bool latched{false};

  void reset() {*this = StartupGuard{};}
  void invalidate_dc() {dc_good_samples = 0; dc_age = kDcMaxAgeCycles + 1;}

  bool step(bool activate, bool feedback_ready, bool bus_ready,
    bool dc_sample, std::uint32_t dc_difference)
  {
    if (!activate) {reset(); return false;}
    if (latched) {return true;}
    if (dc_age <= kDcMaxAgeCycles) {++dc_age;}
    if (!feedback_ready || !bus_ready) {
      bus_cycles = 0;
      invalidate_dc();
      return false;
    }
    if (bus_cycles < kBusCycles) {++bus_cycles;}
    if (dc_sample) {
      dc_age = 0;
      if (dc_difference <= kDcLimitNs) {
        if (dc_good_samples < kDcSamples) {++dc_good_samples;}
      } else {dc_good_samples = 0;}
    }
    if (dc_age > kDcMaxAgeCycles) {dc_good_samples = 0;}
    latched = bus_cycles >= kBusCycles && dc_good_samples >= kDcSamples &&
      dc_age <= kDcMaxAgeCycles;
    return latched;
  }
};

struct EnableSettle
{
  static constexpr unsigned kCycles = 100;
  unsigned cycles{0};
  void reset() {cycles = 0;}
  // Called only in CiA402 Switched On, with the startup gate already open.
  std::uint16_t switched_on_controlword()
  {
    if (cycles < kCycles) {++cycles; return 0x0007;}
    return 0x000f;
  }
};
}  // namespace kaixuan_can_hardware
