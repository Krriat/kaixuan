#include "kaixuan_can_hardware/dc_clock_bridge.hpp"
#include "kaixuan_can_hardware/dc_reference_profile.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace kaixuan_can_hardware;
#define CHECK(c) do {if (!(c)) {std::cerr << __LINE__ << ": " #c "\n"; std::exit(1);}} while (false)

// Two independent host clocks, one physical reference oscillator. Exercise the
// supported MONOTONIC sleep API with repeated RAW deadline rebasing, including
// abrupt NTP rate changes after lock. Do not alter any real system clock.
static DcReferenceFollower::Failure simulate(bool raw_mode, double fast, double slow)
{
  auto config = reference_follow_config();
  DcReferenceFollower follower(config);
  DcReferenceFollower::Output out;
  DcReferenceFollower::Input input;
  std::uint64_t raw = 1000000000, mono = 1000000000, grid = 0;
  bool locked = false;
  for (std::uint64_t cycle = 1; cycle <= 90000; ++cycle) {
    const auto host = raw_mode ? raw : mono;
    const auto base = out.base_ns;
    if (cycle == 1) {grid = host;}
    input.cycle_id = cycle;
    input.now_mono_ns = host;
    input.drives_disabled = true;
    out = follower.step(input);
    if (out.failed) {return out.failure;}
    locked = locked || out.locked;
    if (locked) {CHECK(out.locked);}
    // Twenty microseconds of work before sending the next FRMW/PDO.
    const double rate = 1.0 + (cycle < 30000 ? 0.0 : cycle < 60000 ? fast : slow) / 1e6;
    const auto raw_send = raw + 20000;
    const auto mono_send = mono + static_cast<std::uint64_t>(std::llround(20000 * rate));
    const auto host_send = raw_mode ? raw_send : mono_send;
    input.matched_send_cycle_id = cycle;
    input.host_mono_send_ns = host_send;
    input.host_virtual_send_ns = static_cast<std::uint64_t>(static_cast<std::int64_t>(host_send) - base);
    input.ref32 = static_cast<std::uint32_t>(1000000000 +
      std::llround(double(raw_send - 1000000000) * 1.000047) - 1500000);
    input.valid = true;
    grid += 1000000;
    const auto deadline_host = static_cast<std::uint64_t>(static_cast<std::int64_t>(grid) + out.base_ns);
    std::uint64_t deadline_mono = deadline_host;
    if (raw_mode) {
      CHECK(dc_raw_deadline_to_monotonic(deadline_host, raw_send,
        mono_send, mono_send, deadline_mono));
    }
    CHECK(deadline_mono > mono_send);
    raw = raw_send + static_cast<std::uint64_t>(std::llround(double(deadline_mono - mono_send) / rate));
    mono = deadline_mono;
  }
  CHECK(locked && out.locked && out.accepted_samples > 89900);
  CHECK(std::abs(out.filtered_error_ns) < 5000);
  return out.failure;
}

int main()
{
  std::uint64_t deadline = 0;
  CHECK(!dc_raw_deadline_to_monotonic(10, 10, 20, 20, deadline));
  CHECK(!dc_raw_deadline_to_monotonic(11, 10, 21, 20, deadline));
  CHECK(!dc_raw_deadline_to_monotonic(20, 10,
    std::numeric_limits<std::uint64_t>::max() - 1,
    std::numeric_limits<std::uint64_t>::max(), deadline));
  CHECK(simulate(false, 3500, -2500) == DcReferenceFollower::Failure::SamplesLost);
  CHECK(simulate(true, 3500, -2500) == DcReferenceFollower::Failure::None);
  CHECK(simulate(true, -3500, 2500) == DcReferenceFollower::Failure::None);
  // The reboot trace also observed approximately -14046 ppm during initial
  // Linux clock correction. Cover a larger abrupt change after lock as well.
  CHECK(simulate(true, 15000, -15000) == DcReferenceFollower::Failure::None);
  CHECK(simulate(true, -15000, 15000) == DcReferenceFollower::Failure::None);
  std::cout << "PASS raw clock deadline bridge: four 90s runs survive up to +/-15000ppm discipline; old MONOTONIC mode loses samples\n";
}
