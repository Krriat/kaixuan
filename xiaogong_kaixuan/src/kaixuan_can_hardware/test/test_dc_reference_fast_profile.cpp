#include "kaixuan_can_hardware/dc_reference_profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>

// Small portable regression: no log fixture, filesystem, threads or clocks.
// Uses the same fixed profile as the hardware adapter.
using Follower = kaixuan_can_hardware::DcReferenceFollower;
static std::uint64_t checks = 0;
#define CHECK(expr) do { ++checks; if (!(expr)) { \
  std::cerr << __LINE__ << ": " #expr "\n"; std::exit(1); } } while (false)

using kaixuan_can_hardware::reference_follow_config;

struct Simulation
{
  static constexpr std::uint64_t period = 1000000, origin = 20000000000000ULL;
  const Follower::Config config;
  Follower follower{config};
  Follower::Output result{};
  std::uint64_t cycle{0}, grid{origin}, mono{origin + 30000}, virtual_send{mono};
  std::uint32_t ref32{0};
  double rate_ppm{0}, offset_ns{0}, noise_ns{0};
  mutable std::uint32_t rng{7};
  bool disabled{true}, saw_saturated_seed{false};
  unsigned handoffs{0};
  std::int64_t last_correction{0};
  double fine_raw_peak{0};

  Simulation(double offset = 120000, double ppm = 0,
    Follower::Config profile = reference_follow_config())
  : config(profile), follower(config), rate_ppm(ppm), offset_ns(offset)
  {ref32 = reference(mono);}

  std::uint32_t reference(std::uint64_t host) const
  {
    rng = rng * 1664525U + 1013904223U;
    const double noise = (double(rng) / 4294967295.0 * 2 - 1) * noise_ns;
    return static_cast<std::uint32_t>(std::int64_t(double(origin) +
      double(host - origin) * (1 + rate_ppm * 1e-6) + offset_ns + noise));
  }

  void change_rate(double ppm)
  {
    // Change slope without a discontinuity in the simulated clock phase.
    offset_ns += (rate_ppm - ppm) * 1e-6 * double(mono - origin);
    rate_ppm = ppm;
  }

  void step(bool valid = true, std::optional<std::int64_t> forced_residual = {})
  {
    const auto base = follower.status().base_ns;
    grid += period;
    const auto now = std::uint64_t(std::int64_t(grid) + base + 30000);
    CHECK(now - mono >= 999000 && now - mono <= 1001000);
    Follower::Input in;
    in.cycle_id = ++cycle;
    in.matched_send_cycle_id = cycle - 1;
    in.host_mono_send_ns = mono;
    in.host_virtual_send_ns = virtual_send;
    in.ref32 = forced_residual ?
      std::uint32_t(std::int64_t(mono) - base - *forced_residual) : ref32;
    in.now_mono_ns = now;
    in.valid = valid;
    in.drives_disabled = disabled;
    result = follower.step(in);
    CHECK(result.base_ns - base == result.next_correction_ns);
    CHECK(std::abs(result.next_correction_ns) <= 1000);
    CHECK(std::abs(result.next_correction_ns - last_correction) <= 5);
    CHECK(std::abs(result.frequency_ns_per_cycle) <= 1000);
    if (result.calibrated_this_step) {
      ++handoffs;
      CHECK(disabled && !result.locked && result.calibration_delta_ns == 0);
      const double seed = double(last_correction) -
        config.proportional_gain * result.filtered_error_ns;
      CHECK(std::abs(result.frequency_ns_per_cycle - std::clamp(seed, -1000., 1000.)) < 1e-8);
      saw_saturated_seed = saw_saturated_seed || std::abs(seed) > 1000;
      // A clipped seed is not perfectly bumpless; actual output still has the
      // same hard bound and 5 ns/cycle slew checked above.
    }
    if (result.calibrated && result.residual_available) {
      fine_raw_peak = std::max(fine_raw_peak, std::abs(double(result.residual_ns)));
    }
    if (result.locked) {CHECK(result.disabled_observation_ns >= 10000000000ULL);}
    last_correction = result.next_correction_ns;
    mono = now;
    virtual_send = std::uint64_t(std::int64_t(now) - base); // Frozen entry B.
    ref32 = reference(now);
  }

  void lock()
  {
    while (!result.locked && cycle < 40000) {step(); CHECK(!result.failed);}
    CHECK(result.locked && handoffs == 1 && fine_raw_peak < 100000);
  }

  void check_latched(Follower::Failure expected)
  {
    CHECK(result.failed && !result.locked && result.failure == expected);
    const auto frequency = result.frequency_ns_per_cycle;
    for (unsigned i = 0; i < 100; ++i) {
      step();
      CHECK(result.failed && !result.locked && result.failure == expected);
      CHECK(result.frequency_ns_per_cycle == frequency);
    }
  }
};

static void profile_keeps_protection_and_baseline_defaults()
{
  const Follower::Config baseline;
  const auto fast = reference_follow_config();
  CHECK(!baseline.bumpless_fine_entry && baseline.proportional_gain == .002);
  CHECK(baseline.integral_gain == .000001 && baseline.fine_entry_filtered_ns == 80000);
  CHECK(fast.filter_alpha == baseline.filter_alpha && fast.filter_alpha == 1. / 32.);
  CHECK(fast.raw_limit_ns == 100000 && fast.filtered_limit_ns == 5000);
  CHECK(fast.runtime_bad_quality_samples == 100 && fast.holdover_ns == 20000000);
  CHECK(fast.correction_limit_ns == 1000 && fast.slew_ns == 5);
  CHECK(fast.acquisition_rate_feedforward && !baseline.acquisition_rate_feedforward);
  CHECK(fast.acquisition_timeout_ns == baseline.acquisition_timeout_ns && fast.settle_disabled_ns == baseline.settle_disabled_ns);
}

static void startup_and_bounded_saturated_handoff()
{
  for (int sign : {-1, 1}) {
    Simulation sim(sign * 2000000., sign * 200.);
    sim.lock();
    CHECK(sim.saw_saturated_seed);
    sim.disabled = false;
    for (unsigned i = 0; i < 1000; ++i) {sim.step(); CHECK(sim.result.locked && !sim.result.failed);}
  }
}

static void single_142ppm_rate_change_while_locked()
{
  for (int sign : {-1, 1}) {
    Simulation sim;
    sim.lock();
    sim.disabled = false;
    sim.change_rate(sign * 142.);
    for (unsigned i = 0; i < 3000; ++i) {
      sim.step();
      CHECK(sim.result.locked && !sim.result.failed && sim.result.sample_accepted);
    }
    CHECK(sim.fine_raw_peak < 100000);
  }
}

static void original_quality_protection_stays_latched()
{
  Simulation sim;
  sim.lock();
  sim.disabled = false;
  for (unsigned i = 0; i < 200 && !sim.result.failed; ++i) {sim.step(true, 20000);}
  CHECK(sim.result.bad_quality_samples == 100);
  sim.check_latched(Follower::Failure::QualityLost);
}

static void original_missing_sample_protection_stays_latched()
{
  Simulation sim;
  sim.lock();
  sim.disabled = false;
  for (unsigned i = 0; i < 25 && !sim.result.failed; ++i) {sim.step(false);}
  sim.check_latched(Follower::Failure::SamplesLost);
}

static void acquisition_covers_cold_clock_rates()
{
  for (double offset : {-2000000., -150000., 150000., 2000000.}) {
    for (double rate : {-600., -500., -300., 0., 300., 500., 600.}) {
      for (double noise : {0., 3000.}) {
        for (unsigned seed : {7U, 41U, 8123U}) {
          Simulation sim(offset, rate);
          sim.noise_ns = noise;
          sim.rng = seed;
          sim.lock();
          CHECK(sim.result.acquisition_elapsed_ns <= 25000000000ULL);
        }
      }
    }
  }
  // The old coarse P phase-error floor reproduces error 8 without losing a sample.
  auto legacy = reference_follow_config();
  legacy.acquisition_rate_feedforward = false;
  Simulation old(-1500000., -300., legacy);
  while (old.cycle < 30000 && !old.result.failed) {old.step();}
  CHECK(old.result.failure == Follower::Failure::AcquisitionTimeout);
  CHECK(old.result.rejected_samples == 0 && !old.result.locked);
}

int main()
{
  acquisition_covers_cold_clock_rates();
  profile_keeps_protection_and_baseline_defaults();
  startup_and_bounded_saturated_handoff();
  single_142ppm_rate_change_while_locked();
  original_quality_protection_stays_latched();
  original_missing_sample_protection_stays_latched();
  std::cout << "PASS 6 fast-profile regressions, " << checks
            << " checks; no hardware, clocks, or trace dependency\n";
  return 0;
}
