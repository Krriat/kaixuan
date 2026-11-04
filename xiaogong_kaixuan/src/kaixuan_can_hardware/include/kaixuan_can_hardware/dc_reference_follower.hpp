#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace kaixuan_can_hardware
{
// Offline prototype; single cyclic-thread owner. No IO, clocks, allocation or
// drive commands. A(M)=M-B, P[k+1]=P[k]+period, deadline=P+B.
// Read the previous FRMW result AFTER receive and BEFORE requeueing it. Call
// step once per cycle. It may run before this cycle's send if the caller freezes
// cycle_base=B at entry and uses that frozen base for this entire cycle's send
// timestamp. The returned new B applies only to the NEXT sleep deadline. Calling
// step after send with the same pairing is also supported. Save both host
// timestamps at each actual send; never rewrite an old timestamp using new B.
// Returned base_ns already includes the correction for the NEXT deadline;
// do not add next_correction_ns to that deadline a second time.
class DcReferenceFollower
{
public:
  struct Config {
    std::uint64_t period_ns{1000000};
    std::uint64_t settle_disabled_ns{10000000000ULL};
    std::uint64_t holdover_ns{20000000};
    std::uint64_t acquisition_timeout_ns{25000000000ULL};
    std::int64_t acquisition_limit_ns{20000000};
    unsigned acquisition_qualification_samples{20};
    unsigned fine_entry_good_samples{20};
    std::int64_t fine_entry_filtered_ns{80000};
    std::int64_t fine_entry_correction_ns{400};
    double acquisition_gain{0.004};
    unsigned lock_good_samples{1000};
    unsigned runtime_bad_quality_samples{100};
    std::int64_t frequency_limit_ns{1000};
    std::int64_t correction_limit_ns{1000};
    std::int64_t slew_ns{5};
    std::int64_t raw_limit_ns{100000};
    std::int64_t filtered_limit_ns{5000};
    std::uint64_t reference_interval_tolerance_ns{200000};
    std::uint64_t reference_rate_limit_ppm{2000};
    double filter_alpha{1.0 / 32.0};
    double proportional_gain{0.002};
    double integral_gain{0.000001};
    bool bumpless_fine_entry{false};  // Preserve baseline unless explicitly enabled.
  };
  enum class Failure {
    None, InvalidConfig, NonMonotonicCycle, NonMonotonicTime,
    InitialOffsetOutOfRange, ArithmeticOverflow, SamplesLost, QualityLost,
    AcquisitionTimeout, AcquisitionOffsetOutOfRange
  };
  enum class Reject {
    None, InvalidSample, WrongCycle, SampleAge, PollInterval,
    SendNotIncreasing, ReferenceProgression, ResidualOutOfRange, DisabledRequired
  };
  enum class Quality {AwaitSample, AwaitDisabled, Calibrated, Acquiring, Good, Holdover, Failed,
    CoarseAcquiring};
  struct Input {
    std::uint64_t cycle_id{0};
    std::uint64_t matched_send_cycle_id{0};
    std::uint64_t host_virtual_send_ns{0};
    std::uint64_t host_mono_send_ns{0};
    std::uint32_t ref32{0};  // ecrt_master_reference_clock_time: delay already subtracted.
    std::uint64_t now_mono_ns{0};
    bool valid{false};
    bool drives_disabled{false};  // Fresh domain feedback, CW6, non-enabled, fault-free.
  };
  struct Output {
    std::int64_t base_ns{0};
    std::int64_t next_correction_ns{0};
    std::int64_t calibration_delta_ns{0};  // Compatibility: always zero; no base jumps.
    bool calibrated{false}, calibrated_this_step{false};
    // calibrated_this_step means coarse acquisition finished, not an offset jump.
    bool acquiring_phase{false};
    std::int64_t initial_residual_ns{0};
    std::uint64_t acquisition_elapsed_ns{0}, baseline_restarts{0};
    unsigned coarse_good_samples{0};
    bool locked{false}, failed{false}, sample_accepted{false}, residual_available{false};
    // measurement_error is the matched old virtual timestamp minus ref32.
    // residual compensates for B changes since that old timestamp was made.
    std::int64_t measurement_error_ns{0}, residual_ns{0};
    double filtered_error_ns{0}, frequency_ns_per_cycle{0};
    std::uint64_t sample_age_ns{0}, last_valid_age_ns{0}, disabled_observation_ns{0};
    std::uint64_t accepted_samples{0}, rejected_samples{0};
    unsigned consecutive_good_samples{0}, bad_quality_samples{0};
    Quality quality{Quality::AwaitSample};
    Failure failure{Failure::None};
    Reject rejected_because{Reject::None};
  };

  DcReferenceFollower() : DcReferenceFollower(Config{}) {}
  explicit DcReferenceFollower(const Config & config) : config_(config)
  {
    if (config_.period_ns == 0 || config_.period_ns > 1000000000ULL ||
      config_.holdover_ns < config_.period_ns || config_.holdover_ns >= (1ULL << 31) ||
      config_.acquisition_timeout_ns <= config_.holdover_ns ||
      config_.acquisition_limit_ns <= config_.raw_limit_ns ||
      config_.acquisition_limit_ns >= (1LL << 31) ||
      config_.acquisition_qualification_samples < 2 || config_.fine_entry_good_samples == 0 ||
      config_.fine_entry_filtered_ns <= 0 || config_.fine_entry_filtered_ns >= config_.raw_limit_ns ||
      config_.fine_entry_correction_ns <= 0 || config_.fine_entry_correction_ns > config_.correction_limit_ns ||
      !std::isfinite(config_.acquisition_gain) || config_.acquisition_gain <= 0 ||
      config_.frequency_limit_ns <= 0 || config_.correction_limit_ns < config_.frequency_limit_ns ||
      std::uint64_t(config_.correction_limit_ns) >= config_.period_ns / 2 ||
      config_.slew_ns <= 0 || config_.slew_ns > config_.correction_limit_ns ||
      config_.raw_limit_ns <= 0 || std::uint64_t(config_.raw_limit_ns) >= config_.period_ns / 2 ||
      config_.filtered_limit_ns <= 0 || config_.filtered_limit_ns >= config_.raw_limit_ns ||
      config_.lock_good_samples == 0 || config_.runtime_bad_quality_samples == 0 ||
      config_.reference_rate_limit_ppm > 1000000ULL ||
      config_.reference_interval_tolerance_ns >= config_.period_ns / 2 ||
      !std::isfinite(config_.filter_alpha) || config_.filter_alpha <= 0 || config_.filter_alpha > 1 ||
      !std::isfinite(config_.proportional_gain) || config_.proportional_gain < 0 ||
      !std::isfinite(config_.integral_gain) || config_.integral_gain < 0)
    {
      fail(Failure::InvalidConfig);
    }
  }

  static std::int64_t difference32(std::uint32_t a, std::uint32_t b)
  {
    const std::uint32_t delta = a - b;
    return delta >= 0x80000000U ? std::int64_t(delta) - (1LL << 32) : delta;
  }

  Output status() const {return finish(Output{});}

  Output step(const Input & in)
  {
    Output out;
    if (failure_ == Failure::InvalidConfig) {return finish(out);}
    std::uint64_t interval = 0;
    bool contiguous = true;
    if (have_step_) {
      if (in.cycle_id <= last_cycle_) {fail(Failure::NonMonotonicCycle); return finish(out);}
      if (in.now_mono_ns <= last_now_) {fail(Failure::NonMonotonicTime); return finish(out);}
      interval = in.now_mono_ns - last_now_;
      contiguous = in.cycle_id - last_cycle_ == 1 && in_window(interval);
    }
    // Only confirmed disabled time counts. A gap is not silently counted as
    // time spent disabled, and a single missing reference sample need not reset it.
    if (calibrated_ && contiguous && previous_disabled_ && in.drives_disabled) {
      disabled_ns_ = saturating_add(disabled_ns_, interval);
    }
    have_step_ = true;
    last_cycle_ = in.cycle_id;
    last_now_ = in.now_mono_ns;
    previous_disabled_ = in.drives_disabled;
    if (have_initial_sample_) {
      out.last_valid_age_ns = in.now_mono_ns - last_valid_now_;
      // A fresh sample after an excessive gap cannot erase the outage.
      if (out.last_valid_age_ns > config_.holdover_ns) {fail(Failure::SamplesLost);}
      if (!calibrated_) {
        acquisition_elapsed_ns_ = in.now_mono_ns - acquisition_started_ns_;
        if (acquisition_elapsed_ns_ > config_.acquisition_timeout_ns) {fail(Failure::AcquisitionTimeout);}
      }
    }
    if (failed()) {advance_base(frequency_, out); return finish(out);}

    Reject rejection = Reject::None;
    if (!in.valid) {rejection = Reject::InvalidSample;}
    else if (!contiguous) {rejection = Reject::PollInterval;}
    else if (in.cycle_id == 0 || in.matched_send_cycle_id != in.cycle_id - 1) {
      rejection = Reject::WrongCycle;
    } else if (in.now_mono_ns < in.host_mono_send_ns ||
      !in_window(in.now_mono_ns - in.host_mono_send_ns))
    {
      rejection = Reject::SampleAge;
    } else {
      out.sample_age_ns = in.now_mono_ns - in.host_mono_send_ns;
    }
    if (rejection == Reject::None && have_initial_sample_ && !calibrated_ && !in.drives_disabled) {
      rejection = Reject::DisabledRequired;
    }

    std::int64_t base_at_send = 0, base_change = 0, residual = 0;
    if (rejection == Reject::None) {
      out.measurement_error_ns = difference32(
        static_cast<std::uint32_t>(in.host_virtual_send_ns), in.ref32);
      if (!signed_difference(in.host_mono_send_ns, in.host_virtual_send_ns, base_at_send) ||
        !subtract(base_ns_, base_at_send, base_change) ||
        !subtract(out.measurement_error_ns, base_change, residual))
      {
        fail(Failure::ArithmeticOverflow);
        advance_base(frequency_, out);
        return finish(out);
      }
      out.residual_available = true;
      out.residual_ns = residual;
      if (have_initial_sample_) {
        if (in.host_mono_send_ns <= last_send_mono_ || in.matched_send_cycle_id <= last_send_cycle_) {
          rejection = Reject::SendNotIncreasing;
        } else {
          const auto elapsed = in.host_mono_send_ns - last_send_mono_;
          const std::uint32_t ref_elapsed = in.ref32 - last_ref32_;
          const auto tolerance = config_.reference_interval_tolerance_ns +
            (elapsed < (1ULL << 31) ? elapsed * config_.reference_rate_limit_ppm / 1000000 : 0);
          if (elapsed >= (1ULL << 31) || ref_elapsed >= (1U << 31) ||
            magnitude(std::int64_t(ref_elapsed) - std::int64_t(elapsed)) > tolerance)
          {
            rejection = Reject::ReferenceProgression;
          } else if (calibrated_ && magnitude(residual) >= std::uint64_t(config_.raw_limit_ns)) {
            rejection = Reject::ResidualOutOfRange;
          }
        }
      }
    }

    if (rejection != Reject::None) {
      ++rejected_samples_;
      good_samples_ = 0;
      coarse_good_samples_ = fine_entry_good_samples_ = 0;
      out.rejected_because = rejection;
      quality_ = have_initial_sample_ ? Quality::Holdover : Quality::AwaitSample;
      // A single bad FIRST baseline must not poison all subsequent comparisons.
      // This is allowed only before any nonzero B adjustment. Rebuilding the
      // candidate anchor is NOT an accepted sample and does NOT refresh the
      // last-valid timeout: endless bad baselines still fail after 20 ms.
      if (rejection == Reject::ReferenceProgression && !calibrated_ && !ever_corrected_ &&
        in.drives_disabled && magnitude(residual) <= std::uint64_t(config_.acquisition_limit_ns))
      {
        set_reference_anchor(in);
        initial_residual_ns_ = residual;
        filtered_ = double(residual);
        coarse_good_samples_ = 1;
        ++baseline_restarts_;
      }
      advance_base(frequency_, out);
      return finish(out);
    }

    if (!calibrated_) {
      if (!in.drives_disabled) {quality_ = Quality::AwaitDisabled; return finish(out);}
      // Preserve the complete absolute residual, including whole periods.
      // Unlike a modulo-period phase lock, all of it must actually converge.
      if (magnitude(residual) > std::uint64_t(config_.acquisition_limit_ns)) {
        if (!have_initial_sample_) {initial_residual_ns_ = residual;}
        fail(have_initial_sample_ ? Failure::AcquisitionOffsetOutOfRange : Failure::InitialOffsetOutOfRange);
        advance_base(frequency_, out);
        return finish(out);
      }
      if (!have_initial_sample_) {
        have_initial_sample_ = true;
        acquisition_started_ns_ = in.now_mono_ns;
        initial_residual_ns_ = residual;
        filtered_ = double(residual);
        coarse_good_samples_ = 1;
        quality_ = Quality::CoarseAcquiring;
        accept(in, out);
        return finish(out);  // First sample never changes B.
      }
      accept(in, out);
      filtered_ += config_.filter_alpha * (double(residual) - filtered_);
      coarse_good_samples_ = increment(coarse_good_samples_);
      const bool qualified = coarse_good_samples_ >= config_.acquisition_qualification_samples;
      const bool fine_entry_good = qualified && magnitude(residual) < std::uint64_t(config_.raw_limit_ns) &&
        std::abs(filtered_) <= double(config_.fine_entry_filtered_ns) &&
        magnitude(last_correction_) <= std::uint64_t(config_.fine_entry_correction_ns);
      fine_entry_good_samples_ = fine_entry_good ? increment(fine_entry_good_samples_) : 0;
      if (fine_entry_good_samples_ >= config_.fine_entry_good_samples) {
        calibrated_ = true;
        out.calibrated_this_step = true;
        disabled_ns_ = 0;  // The ten-second fine observation starts HERE.
        good_samples_ = bad_quality_samples_ = 0;
        quality_ = Quality::Acquiring;
        if (config_.bumpless_fine_entry) {
          // Preserve the previous bounded output when switching from coarse P
          // to fine PI. Only seed I; B never jumps and normal bounds/slew remain.
          frequency_ = std::clamp(double(last_correction_) - config_.proportional_gain * filtered_,
            -double(config_.frequency_limit_ns), double(config_.frequency_limit_ns));
          advance_base(frequency_ + config_.proportional_gain * filtered_, out);
        } else {
          advance_base(config_.proportional_gain * filtered_, out);
        }
      } else {
        quality_ = Quality::CoarseAcquiring;
        // No integral during large-error acquisition: prevent windup. A
        // proportional taper brakes before entering the narrow fine PLL.
        advance_base(qualified ? config_.acquisition_gain * filtered_ : 0, out);
      }
      return finish(out);
    }

    accept(in, out);
    filtered_ += config_.filter_alpha * (double(residual) - filtered_);
    frequency_ = std::clamp(frequency_ + config_.integral_gain * filtered_,
      -double(config_.frequency_limit_ns), double(config_.frequency_limit_ns));
    const bool quality_good = std::abs(filtered_) <= double(config_.filtered_limit_ns);
    if (!locked_) {
      good_samples_ = quality_good && in.drives_disabled ? increment(good_samples_) : 0;
      if (in.drives_disabled && disabled_ns_ >= config_.settle_disabled_ns &&
        good_samples_ >= config_.lock_good_samples)
      {
        locked_ = true;
      }
    } else {
      // Normal drive enable does not revoke lock. Only actual clock quality
      // failure or an outage does; the caller owns its controlled-stop action.
      bad_quality_samples_ = quality_good ? 0 : increment(bad_quality_samples_);
      if (bad_quality_samples_ >= config_.runtime_bad_quality_samples) {fail(Failure::QualityLost);}
    }
    quality_ = quality_good ? Quality::Good : Quality::Acquiring;
    const double demand = failed() ? frequency_ :
      frequency_ + config_.proportional_gain * filtered_;
    advance_base(demand, out);
    return finish(out);
  }

private:
  bool failed() const {return failure_ != Failure::None;}
  bool in_window(std::uint64_t ns) const
  {return ns >= config_.period_ns / 2 && ns <= config_.period_ns + config_.period_ns / 2;}
  static std::uint64_t magnitude(std::int64_t value)
  {return value < 0 ? std::uint64_t(-(value + 1)) + 1 : std::uint64_t(value);}
  static unsigned increment(unsigned value)
  {return value == std::numeric_limits<unsigned>::max() ? value : value + 1;}
  static std::uint64_t saturating_add(std::uint64_t a, std::uint64_t b)
  {return b > std::numeric_limits<std::uint64_t>::max() - a ? std::numeric_limits<std::uint64_t>::max() : a + b;}
  static bool signed_difference(std::uint64_t a, std::uint64_t b, std::int64_t & out)
  {
    const auto difference = a >= b ? a - b : b - a;
    if (difference > std::uint64_t(std::numeric_limits<std::int64_t>::max())) {return false;}
    out = a >= b ? std::int64_t(difference) : -std::int64_t(difference);
    return true;
  }
  static bool add(std::int64_t a, std::int64_t b, std::int64_t & out)
  {
    if ((b > 0 && a > std::numeric_limits<std::int64_t>::max() - b) ||
      (b < 0 && a < std::numeric_limits<std::int64_t>::min() - b)) {return false;}
    out = a + b;
    return true;
  }
  static bool subtract(std::int64_t a, std::int64_t b, std::int64_t & out)
  {
    if ((b < 0 && a > std::numeric_limits<std::int64_t>::max() + b) ||
      (b > 0 && a < std::numeric_limits<std::int64_t>::min() + b)) {return false;}
    out = a - b;
    return true;
  }
  void fail(Failure failure)
  {
    if (!failed()) {failure_ = failure;}
    locked_ = false;
    quality_ = Quality::Failed;
  }
  void accept(const Input & in, Output & out)
  {
    ++accepted_samples_;
    last_valid_now_ = in.now_mono_ns;
    set_reference_anchor(in);
    out.sample_accepted = true;
    out.last_valid_age_ns = 0;
  }
  void set_reference_anchor(const Input & in)
  {
    last_send_mono_ = in.host_mono_send_ns;
    last_send_cycle_ = in.matched_send_cycle_id;
    last_ref32_ = in.ref32;
  }
  void advance_base(double demand, Output & out)
  {
    const double bounded = std::clamp(demand,
      -double(config_.correction_limit_ns), double(config_.correction_limit_ns));
    const auto desired = std::int64_t(std::llround(bounded + fraction_));
    const auto limit = config_.correction_limit_ns;
    const auto next = std::clamp(desired,
      std::max(-limit, last_correction_ - config_.slew_ns),
      std::min(limit, last_correction_ + config_.slew_ns));
    fraction_ = desired == next ? bounded + fraction_ - double(next) : 0;
    std::int64_t advanced;
    if (!add(base_ns_, next, advanced)) {fail(Failure::ArithmeticOverflow); return;}
    base_ns_ = advanced;
    ever_corrected_ = ever_corrected_ || next != 0;
    last_correction_ = next;
    out.next_correction_ns = next;
  }
  Output finish(Output out) const
  {
    out.base_ns = base_ns_;
    out.calibrated = calibrated_;
    out.acquiring_phase = have_initial_sample_ && !calibrated_ && !failed();
    out.initial_residual_ns = initial_residual_ns_;
    out.acquisition_elapsed_ns = acquisition_elapsed_ns_;
    out.baseline_restarts = baseline_restarts_;
    out.coarse_good_samples = coarse_good_samples_;
    out.locked = locked_;
    out.failed = failed();
    out.failure = failure_;
    out.quality = failed() ? Quality::Failed : quality_;
    out.filtered_error_ns = filtered_;
    out.frequency_ns_per_cycle = frequency_;
    out.disabled_observation_ns = disabled_ns_;
    out.consecutive_good_samples = good_samples_;
    out.bad_quality_samples = bad_quality_samples_;
    out.accepted_samples = accepted_samples_;
    out.rejected_samples = rejected_samples_;
    return out;
  }

  Config config_;
  std::int64_t base_ns_{0}, last_correction_{0};
  double filtered_{0}, frequency_{0}, fraction_{0};
  bool calibrated_{false}, locked_{false}, have_step_{false}, previous_disabled_{false};
  bool have_initial_sample_{false}, ever_corrected_{false};
  Failure failure_{Failure::None};
  Quality quality_{Quality::AwaitSample};
  std::uint64_t last_cycle_{0}, last_now_{0}, last_valid_now_{0};
  std::uint64_t last_send_mono_{0}, last_send_cycle_{0}, disabled_ns_{0};
  std::uint64_t accepted_samples_{0}, rejected_samples_{0};
  std::uint64_t acquisition_started_ns_{0}, acquisition_elapsed_ns_{0}, baseline_restarts_{0};
  std::int64_t initial_residual_ns_{0};
  std::uint32_t last_ref32_{0};
  unsigned good_samples_{0}, bad_quality_samples_{0};
  unsigned coarse_good_samples_{0}, fine_entry_good_samples_{0};
};
}  // namespace kaixuan_can_hardware
