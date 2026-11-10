#pragma once

#include <cstdint>
#include <limits>

namespace kaixuan_can_hardware
{
// Linux does not support clock_nanosleep(CLOCK_MONOTONIC_RAW). Rebase a RAW
// deadline onto a freshly bracketed MONOTONIC sample on every cycle instead.
// MONOTONIC discipline can then affect only the remaining sub-period wait;
// its accumulated offset never enters the EtherCAT application clock or PLL.
inline bool dc_raw_deadline_to_monotonic(
  std::uint64_t deadline_raw, std::uint64_t now_raw,
  std::uint64_t mono_before, std::uint64_t mono_after,
  std::uint64_t & deadline_mono)
{
  if (mono_after < mono_before || deadline_raw <= now_raw) {return false;}
  const auto midpoint = mono_before + (mono_after - mono_before) / 2;
  const auto remaining = deadline_raw - now_raw;
  if (remaining > std::numeric_limits<std::uint64_t>::max() - midpoint) {return false;}
  deadline_mono = midpoint + remaining;
  return true;
}
}  // namespace kaixuan_can_hardware
