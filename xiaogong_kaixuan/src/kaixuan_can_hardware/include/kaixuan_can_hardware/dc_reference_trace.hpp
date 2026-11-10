#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <ostream>
#include <type_traits>

namespace kaixuan_can_hardware
{
// One writer during cyclic operation; readers only after that thread joins.
// Allocate and touch the whole object during configure, never on the RT stack.
class DcReferenceTraceRing
{
public:
  static constexpr std::size_t kCapacity = 8192;
  enum Flag : std::uint32_t {
    PreviousSendQueued = 1U << 0, GetterCalled = 1U << 1,
    GetterSucceeded = 1U << 2, SampleAccepted = 1U << 3,
    ResidualAvailable = 1U << 4, PreviousRawValid = 1U << 5,
    CurrentRawValid = 1U << 6, Calibrated = 1U << 7,
    Locked = 1U << 8, Acquiring = 1U << 9,
    FollowerFailed = 1U << 10, GetterFailed = 1U << 11,
    ScheduleFailed = 1U << 12, StartupReady = 1U << 13,
    AllControl6 = 1U << 14, AllControl15 = 1U << 15,
    AllDisabledFeedback = 1U << 16, AllEnabledFeedback = 1U << 17,
    ClockReadError = 1U << 18
  };
  struct RawBracket {
    std::uint64_t mono_before_ns{0}, raw_ns{0}, mono_after_ns{0};
    bool valid{false};
  };
  struct Row {
    std::uint64_t cycle{0}, wake_mono_ns{0}, scheduled_mono_ns{0};
    std::uint64_t application_ns{0}, previous_send_cycle{0};
    std::uint64_t wake_host_ns{0}, previous_send_host_ns{0}, next_host_deadline_ns{0};
    std::uint64_t previous_send_mono_ns{0}, previous_send_virtual_ns{0};
    RawBracket previous_raw{}, current_raw{};
    std::uint64_t getter_before_ns{0}, getter_after_ns{0};
    std::uint64_t send_before_ns{0}, send_after_ns{0}, next_deadline_ns{0};
    std::uint64_t sample_age_ns{0}, last_valid_age_ns{0}, first_failure_cycle{0};
    std::uint64_t receive_end_ns{0}, exchange_end_ns{0}, mutex_wait_ns{0};
    std::int64_t base_entry_ns{0}, base_after_ns{0};
    std::int64_t measurement_ns{0}, residual_ns{0}, correction_ns{0};
    double filtered_ns{0}, frequency_ns_per_cycle{0};
    std::uint32_t ref32{0}, flags{0}, quality{0}, failure{0}, rejection{0};
    std::uint32_t working_counter{0}, wc_state{0}, bad_quality_samples{0};
    std::int32_t getter_result{0}, slave_sync_result{0}, send_result{0};
    std::array<std::uint16_t, 7> control{}, status{}, error{};
  };
  static_assert(std::is_trivially_copyable<Row>::value, "RT row must remain a plain value");

  DcReferenceTraceRing()
  {
    // Volatile stores guarantee every page is faulted in before the RT thread
    // starts, even if the compiler elides value initialization of untouched rows.
    volatile unsigned char * bytes = reinterpret_cast<volatile unsigned char *>(rows_.data());
    for (std::size_t i = 0; i < sizeof(rows_); ++i) {bytes[i] = 0;}
  }
  bool push(const Row & row, bool failure) noexcept
  {
    if (frozen_) {return false;}
    rows_[next_] = row;
    next_ = (next_ + 1) % kCapacity;
    if (size_ < kCapacity) {++size_;}
    else {++overwritten_;}
    ++total_;
    if (failure) {frozen_ = true; first_failure_cycle_ = row.cycle;}
    return true;
  }
  bool frozen() const noexcept {return frozen_;}
  std::size_t size() const noexcept {return size_;}
  std::uint64_t total() const noexcept {return total_;}
  std::uint64_t overwritten() const noexcept {return overwritten_;}
  std::uint64_t first_failure_cycle() const noexcept {return first_failure_cycle_;}
  // index is chronological and must be less than size(). No RT reader uses it.
  const Row & chronological(std::size_t index) const noexcept
  {return rows_[((next_ + kCapacity - size_) + index) % kCapacity];}

  // NON-RT only, after the writer joins. No file is opened by the ring itself.
  void write_csv(std::ostream & out, unsigned master_id) const
  {
    out << std::setprecision(17)
      << "# schema,dc_reference_trace_v2\n# master," << master_id
      << "\n# capacity," << kCapacity << "\n# row_bytes," << sizeof(Row)
      << "\n# retained_rows," << size_ << "\n# total_rows," << total_
      << "\n# overwritten_rows," << overwritten_ << "\n# truncated," << (overwritten_ != 0)
      << "\n# frozen," << frozen_ << "\n# first_failure_cycle," << first_failure_cycle_
      << "\n# extra_clock_reads_per_active_cycle,3_RAW_bracket_plus_2_if_getter_called"
      << "\n# raw_bracket_location,before_original_control_send_start"
      << "\n# original_control_send_timestamp,unchanged_immediately_before_ecrt_master_send"
      << "\n# freeze_policy,include_first_failure_cycle_then_stop_rows_and_extra_clock_reads"
      << "\n# previous_raw_pairing,same_previous_send_cycle_as_Msend_and_A_send"
      << "\n# getter_not_called,GetterCalled_flag_clear_and_getter_timestamps_zero"
      << "\n# residual_validity,only_use_when_ResidualAvailable_flag_set"
      << "\n# pdo_semantics,control_is_current_output_status_and_error_are_last_received_input"
      << "\n# quality_enum,0AwaitSample_1AwaitDisabled_2Calibrated_3Acquiring_4Good_5Holdover_6Failed_7CoarseAcquiring"
      << "\n# flag_bits,0PreviousSendQueued_1GetterCalled_2GetterSucceeded_3SampleAccepted_4ResidualAvailable_5PreviousRawValid_6CurrentRawValid_7Calibrated_8Locked_9Acquiring_10FollowerFailed_11GetterFailed_12ScheduleFailed_13StartupReady_14AllControl6_15AllControl15_16AllDisabledFeedback_17AllEnabledFeedback_18ClockReadError\n"
      << "cycle,wake_mono_ns,scheduled_mono_ns,application_ns,quality,flags,helper_failure,helper_rejection,first_failure_cycle,"
      << "previous_send_cycle,previous_send_mono_ns,previous_send_virtual_ns,ref32,"
      << "previous_raw_valid,previous_raw_mono_before_ns,previous_raw_ns,previous_raw_mono_after_ns,"
      << "base_entry_ns,base_after_ns,measurement_ns,residual_ns,filtered_ns,frequency_ns_per_cycle,correction_ns,"
      << "sample_age_ns,last_valid_age_ns,bad_quality_samples,getter_before_mono_ns,getter_after_mono_ns,getter_result,"
      << "receive_end_ns,exchange_end_ns,mutex_wait_ns,slave_sync_result,send_result,send_before_mono_ns,send_after_mono_ns,"
      << "current_raw_valid,current_raw_mono_before_ns,current_raw_ns,current_raw_mono_after_ns,next_deadline_ns,working_counter,wc_state";
    for (unsigned axis = 0; axis < 7; ++axis) {
      out << ",axis" << axis << "_control,axis" << axis << "_status,axis" << axis << "_error";
    }
    out << ",wake_host_ns,previous_send_host_ns,next_host_deadline_ns\n";
    for (std::size_t i = 0; i < size_; ++i) {
      const auto & r = chronological(i);
      out << r.cycle << ',' << r.wake_mono_ns << ',' << r.scheduled_mono_ns << ',' << r.application_ns
        << ',' << r.quality << ',' << r.flags << ',' << r.failure << ',' << r.rejection << ',' << r.first_failure_cycle
        << ',' << r.previous_send_cycle << ',' << r.previous_send_mono_ns << ',' << r.previous_send_virtual_ns << ',' << r.ref32
        << ',' << r.previous_raw.valid << ',' << r.previous_raw.mono_before_ns << ',' << r.previous_raw.raw_ns << ',' << r.previous_raw.mono_after_ns
        << ',' << r.base_entry_ns << ',' << r.base_after_ns << ',' << r.measurement_ns << ',' << r.residual_ns
        << ',' << r.filtered_ns << ',' << r.frequency_ns_per_cycle << ',' << r.correction_ns
        << ',' << r.sample_age_ns << ',' << r.last_valid_age_ns << ',' << r.bad_quality_samples
        << ',' << r.getter_before_ns << ',' << r.getter_after_ns << ',' << r.getter_result
        << ',' << r.receive_end_ns << ',' << r.exchange_end_ns << ',' << r.mutex_wait_ns
        << ',' << r.slave_sync_result << ',' << r.send_result << ',' << r.send_before_ns << ',' << r.send_after_ns
        << ',' << r.current_raw.valid << ',' << r.current_raw.mono_before_ns << ',' << r.current_raw.raw_ns << ',' << r.current_raw.mono_after_ns
        << ',' << r.next_deadline_ns << ',' << r.working_counter << ',' << r.wc_state;
      for (unsigned axis = 0; axis < 7; ++axis) {
        out << ',' << r.control[axis] << ',' << r.status[axis] << ',' << r.error[axis];
      }
      out << ',' << r.wake_host_ns << ',' << r.previous_send_host_ns << ',' << r.next_host_deadline_ns << '\n';
    }
  }

private:
  std::array<Row, kCapacity> rows_{};
  std::size_t next_{0}, size_{0};
  std::uint64_t total_{0}, overwritten_{0}, first_failure_cycle_{0};
  bool frozen_{false};
};
}  // namespace kaixuan_can_hardware
