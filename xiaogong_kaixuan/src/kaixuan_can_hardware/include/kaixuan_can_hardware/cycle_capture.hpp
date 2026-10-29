#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstddef>

namespace kaixuan_can_hardware
{
// Version 1: 14 master fields + 7 axes * 9 fields. Signed 64-bit wire values.
// Master: cycle, sample_ns, scheduled_ns, lateness_ns, interval_ns, WKC, WC_state,
// previous_send_result, previous_send_duration_ns, dropped_frames, AL_states,
// slaves_responding, link_up, master_state_result.
// Axis: actual_counts, target_counts, velocity_raw, torque_raw, status_word,
// control_word, error_code, transition_flags, mode_command.
constexpr std::size_t kCaptureHeader = 14;
constexpr std::size_t kCaptureAxisWidth = 9;
constexpr std::size_t kCaptureWidth = kCaptureHeader + 7 * kCaptureAxisWidth;
using CycleFrame = std::array<std::int64_t, kCaptureWidth>;

// Single cyclic producer and single ROS consumer per master. Never waits for
// the consumer and never allocates in push/pop. Overflow is explicitly counted.
template<std::size_t Capacity = 2048>
class CycleCapture
{
public:
  static_assert(Capacity > 1);
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
  bool push(const CycleFrame & frame)
  {
    const auto write = write_.load(std::memory_order_relaxed);
    if (write - read_.load(std::memory_order_acquire) == Capacity) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    frames_[write % Capacity] = frame;
    write_.store(write + 1, std::memory_order_release);
    return true;
  }
  bool pop(CycleFrame & frame)
  {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) {return false;}
    frame = frames_[read % Capacity];
    read_.store(read + 1, std::memory_order_release);
    return true;
  }
  std::uint64_t dropped() const {return dropped_.load(std::memory_order_relaxed);}
  // Only when both producer and consumer are stopped.
  void reset() {write_ = 0; read_ = 0; dropped_ = 0;}

private:
  std::array<CycleFrame, Capacity> frames_{};
  alignas(64) std::atomic<std::uint64_t> write_{0};
  alignas(64) std::atomic<std::uint64_t> read_{0};
  std::atomic<std::uint64_t> dropped_{0};
};

inline std::int64_t capture_transition(
  std::uint16_t old_status, std::uint16_t status,
  std::uint16_t old_control, std::uint16_t control,
  std::uint16_t old_error, std::uint16_t error, bool error_rising)
{
  // Ignore target-reached bit chatter here; every full status word is retained.
  return ((old_status ^ status) & 0x00EF ? 1 : 0) |
         (old_control != control ? 2 : 0) | (old_error != error ? 4 : 0) |
         (error_rising ? 8 : 0);
}
}  // namespace kaixuan_can_hardware
