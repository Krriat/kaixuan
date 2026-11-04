#include "kaixuan_can_hardware/dc_reference_follower.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

using Follower = kaixuan_can_hardware::DcReferenceFollower;
static std::uint64_t assertions = 0;
#define CHECK(condition) do { ++assertions; if (!(condition)) { \
  std::cerr << __FILE__ << ':' << __LINE__ << " failed: " #condition << '\n'; \
  std::exit(1); } } while (false)

struct Fixture {
  static constexpr std::uint64_t T = 1000000, origin = 20000000000000ULL;
  Follower follower;
  Follower::Output output{};
  std::uint64_t cycle{0}, grid{origin};
  std::uint64_t previous_mono{origin + 30000}, previous_virtual{origin + 30000};
  std::uint32_t previous_ref{0}, rng{7};
  double ppm{0}, offset{120000}, noise_amplitude{0};
  std::int64_t previous_correction{0};
  unsigned calibration_count{0};
  bool disabled{true};

  explicit Fixture(Follower::Config config = Follower::Config{}) : follower(config)
  {previous_ref = reference(previous_mono);}

  std::uint32_t reference(std::uint64_t mono)
  {
    rng = rng * 1664525U + 1013904223U;
    const double noise = (double(rng) / double(std::numeric_limits<std::uint32_t>::max()) * 2 - 1) * noise_amplitude;
    const auto value = std::int64_t(double(origin) + double(mono-origin) * (1 + ppm * 1e-6) + offset + noise);
    return static_cast<std::uint32_t>(value);
  }

  Follower::Input next(bool valid = true)
  {
    const auto base = follower.status().base_ns;
    ++cycle; grid += T;
    const auto current_mono = std::uint64_t(std::int64_t(grid) + base + 30000);
    CHECK(current_mono-previous_mono>=999000 && current_mono-previous_mono<=1001000);
    Follower::Input input;
    input.cycle_id = cycle;
    input.matched_send_cycle_id = cycle - 1;
    input.host_mono_send_ns = previous_mono;
    input.host_virtual_send_ns = previous_virtual;
    input.ref32 = previous_ref;
    input.now_mono_ns = current_mono;
    input.valid = valid;
    input.drives_disabled = disabled;
    // Freeze base at entry even if the helper is called before this send.
    previous_mono = current_mono;
    previous_virtual = std::uint64_t(std::int64_t(current_mono) - base);
    previous_ref = reference(current_mono);
    return input;
  }

  Follower::Output tick(Follower::Input input)
  {
    const auto before = follower.status().base_ns;
    output = follower.step(input);
    CHECK(std::abs(output.frequency_ns_per_cycle) <= 1000.000001);
    CHECK(std::abs(output.next_correction_ns) <= 1000);
    if (input.cycle_id == cycle) {
      CHECK(std::abs(output.next_correction_ns - previous_correction) <= 5);
    }
    if (output.calibrated_this_step) {
      ++calibration_count;
      CHECK(output.calibration_delta_ns==0);
      CHECK(input.drives_disabled && !output.locked);
    }
    CHECK(output.base_ns - before == output.next_correction_ns);
    previous_correction = output.next_correction_ns;
    return output;
  }

  Follower::Output tick(bool valid = true) {return tick(next(valid));}

  void force_residual(Follower::Input & input, std::int64_t residual)
  {
    input.ref32 = static_cast<std::uint32_t>(std::int64_t(input.host_mono_send_ns) -
      follower.status().base_ns - residual);
  }

  void lock(unsigned maximum = 60000)
  {
    while (!output.locked && cycle < maximum) {
      tick(); CHECK(!output.failed);
    }
    CHECK(output.locked && output.disabled_observation_ns >= 10000000000ULL);
  }
};

void awaiting_and_one_time_calibration()
{
  Fixture f;
  for (int i=0; i<5000; ++i) {f.tick(false); CHECK(!f.output.failed && !f.output.calibrated && f.output.base_ns==0);}
  f.disabled=false; f.tick();
  CHECK(f.output.quality==Follower::Quality::AwaitDisabled && !f.output.calibrated);
  f.disabled=true; f.tick();
  CHECK(f.output.acquiring_phase && f.output.initial_residual_ns==-120000 && f.output.base_ns==0);
  for (int i=0; i<18; ++i) {f.tick(); CHECK(f.output.base_ns==0);}
  f.tick(); CHECK(f.output.next_correction_ns==-5 && !f.output.calibrated);
  f.tick();
  CHECK(f.output.measurement_error_ns==-120000 && f.output.residual_ns==-119995);
  while (!f.output.calibrated && f.cycle<10000) {f.tick(); CHECK(!f.output.failed);}
  CHECK(f.output.calibrated && f.calibration_count==1 && f.output.calibration_delta_ns==0);
  for (auto error : {-20000000LL,-2000000LL,-500000LL,-1LL,0LL,1LL,500000LL,2000000LL,20000000LL}) {
    Fixture x; auto input=x.next(); x.force_residual(input,error); x.tick(input);
    CHECK(x.output.acquiring_phase && !x.output.calibrated && x.output.base_ns==0);
    CHECK(x.output.initial_residual_ns==error && x.output.calibration_delta_ns==0);
  }
  for (auto error : {-20000001LL,20000001LL}) {
    Fixture x; auto input=x.next(); x.force_residual(input,error); x.tick(input);
    CHECK(x.output.failed && x.output.failure==Follower::Failure::InitialOffsetOutOfRange);
    CHECK(!x.output.calibrated && x.output.base_ns==0);
  }
}

void exact_pairing_and_age_checks()
{
  for (int scenario=0; scenario<6; ++scenario) {
    Fixture f; f.tick(); auto input=f.next();
    if (scenario==0) {input.matched_send_cycle_id=input.cycle_id;}
    if (scenario==1) {input.matched_send_cycle_id=0; input.cycle_id=3;}
    if (scenario==2) {input.host_mono_send_ns=input.now_mono_ns-499999;}
    if (scenario==3) {input.host_mono_send_ns=input.now_mono_ns-1500001;}
    if (scenario==4) {input.host_mono_send_ns=input.now_mono_ns+1;}
    if (scenario==5) {input.valid=false;}
    f.tick(input);
    CHECK(!f.output.sample_accepted && f.output.rejected_because!=Follower::Reject::None);
    CHECK(!f.output.failed && !f.output.locked);
  }
  Fixture f; f.tick();
  while (f.output.base_ns==0) {f.tick();}
  auto wrong=f.next(); wrong.ref32-=1000000; f.tick(wrong);
  CHECK(!f.output.sample_accepted);  // Never hide this using modulo 1 ms.
  CHECK(f.output.rejected_because==Follower::Reject::ReferenceProgression ||
    f.output.rejected_because==Follower::Reject::ResidualOutOfRange);
  f.tick(); CHECK(f.output.sample_accepted && !f.output.failed);
}

void arithmetic_wrap_and_progression()
{
  CHECK(Follower::difference32(0x20,0xfffffff0)==48);
  CHECK(Follower::difference32(0xfffffff0,0x20)==-48);
  CHECK(Follower::difference32(0x80000000,0)==-(1LL<<31));
  CHECK(Follower::difference32(2001234,1001234)==1000000);
  std::uint32_t a=0xace12345;
  for (unsigned i=0; i<100000; ++i) {
    a=a*1664525U+1013904223U;
    const auto delta=std::int64_t(i%1001)-500;
    const auto b=std::uint32_t(a-std::uint32_t(delta));
    CHECK(Follower::difference32(a,b)==delta);
  }
  Fixture f; f.tick(); auto input=f.next(); input.ref32+=300000; f.tick(input);
  CHECK(!f.output.sample_accepted && f.output.rejected_because==Follower::Reject::ReferenceProgression);
  Fixture overflow; auto bad=overflow.next();
  bad.now_mono_ns=std::numeric_limits<std::uint64_t>::max();
  bad.host_mono_send_ns=bad.now_mono_ns-Fixture::T; bad.host_virtual_send_ns=0;
  overflow.tick(bad); CHECK(overflow.output.failure==Follower::Failure::ArithmeticOverflow);
}

void settling_and_enabled_lock_latch()
{
  Fixture f; f.tick();
  while (!f.output.calibrated) {f.tick(); CHECK(f.output.disabled_observation_ns==0 && !f.output.locked);}
  while (f.output.disabled_observation_ns<10000000000ULL) {
    CHECK(!f.output.locked); f.tick(); CHECK(!f.output.failed);
  }
  CHECK(f.output.locked);
  f.disabled=false;
  for (int i=0; i<1000; ++i) {f.tick(); CHECK(f.output.locked && !f.output.failed);}
  f.disabled=true;
  for (int i=0; i<100; ++i) {f.tick(); CHECK(f.output.locked && f.calibration_count==1);}
  Fixture interrupted; interrupted.tick();
  while (!interrupted.output.calibrated) {interrupted.tick();}
  for (int i=0; i<5000; ++i) {interrupted.tick();}
  const auto observed=interrupted.output.disabled_observation_ns;
  interrupted.disabled=false;
  for (int i=0; i<3000; ++i) {interrupted.tick(); CHECK(!interrupted.output.locked);}
  CHECK(interrupted.output.disabled_observation_ns==observed);
  interrupted.disabled=true;
  interrupted.tick(); CHECK(interrupted.output.disabled_observation_ns==observed);
  while (interrupted.output.disabled_observation_ns<10000000000ULL) {
    CHECK(!interrupted.output.locked); interrupted.tick();
  }
  CHECK(interrupted.output.locked);
}

void dropout_holdover_and_latched_failure()
{
  Fixture f; f.lock(); f.disabled=false;
  const double frequency=f.output.frequency_ns_per_cycle;
  for (int i=0; i<10; ++i) {f.tick(false); CHECK(f.output.locked && !f.output.failed && f.output.frequency_ns_per_cycle==frequency);}
  f.tick(); CHECK(f.output.sample_accepted && f.output.locked);
  for (int i=0; i<20; ++i) {f.tick(false); CHECK(!f.output.failed && f.output.locked);}
  f.tick(false);
  CHECK(f.output.failed && !f.output.locked && f.output.failure==Follower::Failure::SamplesLost);
  const auto held=f.output.frequency_ns_per_cycle;
  for (int i=0; i<100; ++i) {f.tick(); CHECK(f.output.failed && !f.output.locked && f.output.frequency_ns_per_cycle==held);}
  Fixture one; one.lock(); one.tick(false); one.tick();
  CHECK(one.output.sample_accepted && one.output.locked && one.calibration_count==1);
  // A fresh timestamp after a >20ms step gap must not conceal that gap.
  Fixture long_gap; long_gap.tick(); auto late=long_gap.next();
  late.now_mono_ns+=20000001; late.host_mono_send_ns=late.now_mono_ns-Fixture::T;
  long_gap.tick(late); CHECK(long_gap.output.failed && long_gap.output.failure==Follower::Failure::SamplesLost);
}

void runtime_quality_loss()
{
  Fixture f; f.lock(); f.disabled=false;
  for (int i=0; i<140 && !f.output.failed; ++i) {
    auto input=f.next(); f.force_residual(input,20000); f.tick(input);
  }
  CHECK(f.output.failed && f.output.failure==Follower::Failure::QualityLost && !f.output.locked);
  const auto frequency=f.output.frequency_ns_per_cycle;
  for (int i=0; i<50; ++i) {f.tick(); CHECK(f.output.failed && f.output.frequency_ns_per_cycle==frequency);}
}

void frequency_correction_and_integer_slew_limits()
{
  for (int direction : {-1,1}) {
    Follower::Config cfg; cfg.integral_gain=1; cfg.proportional_gain=1; cfg.filter_alpha=1;
    Fixture f(cfg); auto first=f.next(); f.force_residual(first,0); f.tick(first);
    for (int i=0; i<250; ++i) {
      auto input=f.next(); f.force_residual(input,direction*4000); f.tick(input);
      CHECK(!f.output.failed);
    }
    CHECK(f.output.frequency_ns_per_cycle==direction*1000);
    CHECK(f.output.next_correction_ns==direction*1000);
    const auto base=f.output.base_ns;
    for (int i=0; i<40; ++i) {f.tick(false);}
    CHECK(f.output.failed && f.output.base_ns-base==direction*40000);
  }
}

void time_cycle_discontinuities_and_invalid_config()
{
  Fixture duplicate; duplicate.tick(); auto input=duplicate.next(); input.cycle_id=1;
  const auto before=duplicate.output.base_ns;
  const auto output=duplicate.follower.step(input);
  CHECK(output.failure==Follower::Failure::NonMonotonicCycle && output.base_ns==before);
  Fixture backwards; backwards.tick(); auto reverse=backwards.next(); reverse.now_mono_ns-=2000000;
  CHECK(backwards.follower.step(reverse).failure==Follower::Failure::NonMonotonicTime);
  Fixture jump; jump.tick(); auto later=jump.next(); later.cycle_id+=1;
  jump.tick(later); CHECK(!jump.output.failed && jump.output.rejected_because==Follower::Reject::PollInterval);
  for (int kind=0; kind<5; ++kind) {
    Follower::Config cfg;
    if (kind==0) {cfg.period_ns=0;}
    if (kind==1) {cfg.slew_ns=0;}
    if (kind==2) {cfg.holdover_ns=1ULL<<31;}
    if (kind==3) {cfg.integral_gain=std::numeric_limits<double>::quiet_NaN();}
    if (kind==4) {cfg.correction_limit_ns=500000;}
    Follower invalid(cfg); CHECK(invalid.status().failure==Follower::Failure::InvalidConfig);
  }
}

void drift_noise_wraps_and_before_after_send_equivalence()
{
  for (double drift : {-100.0,0.0,100.0}) {
    Fixture f; f.ppm=drift; f.noise_amplitude=30000;
    f.previous_ref=f.reference(f.previous_mono);
    unsigned wraps=0; std::uint32_t last_ref=f.previous_ref;
    for (int i=0; i<60000; ++i) {
      auto input=f.next();
      if (last_ref>0xf0000000U && input.ref32<0x10000000U) {++wraps;}
      last_ref=input.ref32;
      f.tick(input); CHECK(!f.output.failed);
    }
    CHECK(f.output.locked && wraps>=13 && f.calibration_count==1);
    CHECK(std::abs(f.output.frequency_ns_per_cycle + drift)<15);
    std::cout << "drift=" << drift << "ppm final_frequency=" << f.output.frequency_ns_per_cycle
              << "ns/cycle wraps=" << wraps << '\n';
  }
  Fixture a; Follower before_send;
  for (int i=0; i<12000; ++i) {
    auto input=a.next(); auto earlier=input; earlier.now_mono_ns-=30000;
    const auto first=before_send.step(earlier); a.tick(input);
    CHECK(first.base_ns==a.output.base_ns && first.frequency_ns_per_cycle==a.output.frequency_ns_per_cycle);
    CHECK(first.locked==a.output.locked && first.failed==a.output.failed);
  }
}

void millisecond_offsets_bounded_acquisition_and_fine_gate()
{
  for (double offset : {-20000000.0,-2000000.0,2000000.0,20000000.0}) {
    Fixture f; f.offset=offset; f.previous_ref=f.reference(f.previous_mono); f.tick();
    CHECK(f.output.base_ns==0 && f.output.initial_residual_ns==-std::int64_t(offset));
    while (!f.output.calibrated && f.cycle<25002) {
      f.tick(); CHECK(!f.output.failed && !f.output.locked && f.output.disabled_observation_ns==0);
    }
    CHECK(f.output.calibrated && f.output.acquisition_elapsed_ns<25000000000ULL);
    CHECK(std::abs(f.output.residual_ns)<100000 && std::abs(f.output.filtered_error_ns)<=80000);
    CHECK(f.calibration_count==1);
    const auto acquisition_cycle=f.cycle;
    f.lock();
    CHECK(f.cycle-acquisition_cycle>=9990);  // Coarse time never counts toward fine observation.
    CHECK(std::abs(double(f.output.base_ns)+offset)<5000);
    std::cout << "initial=" << -offset << "ns acquisition=" << acquisition_cycle/1000.0
              << "s locked=" << f.cycle/1000.0 << "s final_base_error="
              << double(f.output.base_ns)+offset << "ns\n";
  }
  Fixture drifting; drifting.offset=2000000; drifting.ppm=100; drifting.noise_amplitude=30000;
  drifting.previous_ref=drifting.reference(drifting.previous_mono);
  drifting.lock(); CHECK(drifting.output.locked && !drifting.output.failed);
  for (double offset : {-20000000.0,-19000000.0,19000000.0,20000000.0}) {
    for (double drift : {-100.0,100.0}) {
      Fixture f; f.offset=offset-30000*drift*1e-6; f.ppm=drift;
      f.previous_ref=f.reference(f.previous_mono);
      const bool outward_at_limit=std::abs(offset)==20000000 && offset*drift>0;
      double post_entry_max=0;
      while ((!f.output.calibrated || f.output.disabled_observation_ns<2000000000ULL) && f.cycle<30000) {
        f.tick();
        if (outward_at_limit && f.output.failed) {break;}
        CHECK(!f.output.failed);
        if (f.output.calibrated && f.output.residual_available) {
          post_entry_max=std::max(post_entry_max,std::abs(double(f.output.residual_ns)));
        }
      }
      if (outward_at_limit) {
        CHECK(f.output.failure==Follower::Failure::AcquisitionOffsetOutOfRange);
        CHECK(f.output.base_ns==0 && !f.output.calibrated);
        std::cout << "initial_at_limit=" << -offset << "ns outward_drift=" << drift
                  << "ppm correctly rejected before any B correction\n";
        continue;
      }
      CHECK(f.output.calibrated && f.output.acquisition_elapsed_ns<25000000000ULL);
      CHECK(post_entry_max<100000);
      std::cout << "large_initial=" << -offset << "ns drift=" << drift
                << "ppm acquire=" << f.output.acquisition_elapsed_ns/1e9
                << "s fine_peak=" << post_entry_max << "ns\n";
    }
  }
}

void bad_initial_baseline_restarts_without_hiding_outage()
{
  Fixture f; f.offset=2000000; f.previous_ref=f.reference(f.previous_mono);
  auto bad=f.next(); bad.ref32+=500000; f.tick(bad);
  CHECK(f.output.base_ns==0 && f.output.initial_residual_ns==-2500000);
  f.tick();
  CHECK(!f.output.sample_accepted && f.output.baseline_restarts==1 && f.output.base_ns==0);
  CHECK(f.output.initial_residual_ns==-2000000 && f.output.last_valid_age_ns>0);
  for (int i=0; i<18; ++i) {f.tick(); CHECK(f.output.base_ns==0);}
  f.tick(); CHECK(f.output.next_correction_ns==-5);
  // Once B has changed, a reference discontinuity cannot silently re-anchor.
  auto discontinuity=f.next(); discontinuity.ref32+=1000000; f.tick(discontinuity);
  CHECK(!f.output.sample_accepted && f.output.baseline_restarts==1);
  Fixture endless;
  for (int i=0; i<25 && !endless.output.failed; ++i) {
    auto input=endless.next(); input.ref32+=i%2 ? 1000000 : 0; endless.tick(input);
  }
  CHECK(endless.output.failed && endless.output.failure==Follower::Failure::SamplesLost);
  CHECK(endless.output.base_ns==0 && endless.output.baseline_restarts>1);
}

void coarse_disable_requirement_and_timeout()
{
  Fixture f; f.offset=2000000; f.previous_ref=f.reference(f.previous_mono);
  for (int i=0; i<100; ++i) {f.tick();}
  CHECK(!f.output.calibrated && f.output.next_correction_ns<0);
  auto previous=f.output.next_correction_ns;
  f.disabled=false;
  for (int i=0; i<10; ++i) {
    f.tick(); CHECK(f.output.rejected_because==Follower::Reject::DisabledRequired);
    CHECK(std::abs(f.output.next_correction_ns)<=std::abs(previous));
    CHECK(f.output.disabled_observation_ns==0 && !f.output.failed);
    previous=f.output.next_correction_ns;
  }
  f.disabled=true; f.tick(); CHECK(f.output.sample_accepted && f.output.coarse_good_samples==1);
  Fixture timeout;
  for (int i=0; i<25010 && !timeout.output.failed; ++i) {
    auto input=timeout.next(); timeout.force_residual(input,2000000); timeout.tick(input);
  }
  CHECK(timeout.output.failure==Follower::Failure::AcquisitionTimeout && !timeout.output.calibrated);
  Fixture too_far; auto first=too_far.next(); too_far.force_residual(first,20000000); too_far.tick(first);
  auto second=too_far.next(); too_far.force_residual(second,20000001); too_far.tick(second);
  CHECK(too_far.output.failure==Follower::Failure::AcquisitionOffsetOutOfRange);
}

int main()
{
  awaiting_and_one_time_calibration();
  exact_pairing_and_age_checks();
  arithmetic_wrap_and_progression();
  settling_and_enabled_lock_latch();
  dropout_holdover_and_latched_failure();
  runtime_quality_loss();
  frequency_correction_and_integer_slew_limits();
  time_cycle_discontinuities_and_invalid_config();
  drift_noise_wraps_and_before_after_send_equivalence();
  millisecond_offsets_bounded_acquisition_and_fine_gate();
  bad_initial_baseline_restarts_without_hiding_outage();
  coarse_disable_requirement_and_timeout();
  std::cout << "PASS 12 test groups, " << assertions << " assertions; no hardware/clock access\n";
}
