#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "kaixuan_can_hardware/cycle_capture.hpp"

namespace kaixuan_can_hardware
{
// Read-only jobs. KGU V1.2 pp. 15-17, 30, 56-58; optional standard CoE
// objects are probes, NOT assumed to exist in this KGU firmware.
struct ProbeSpec
{
  unsigned kind;  // 0 = ESC register, 1 = CoE upload (never download)
  std::uint16_t index;
  std::uint8_t sub;
  std::size_t size;
  unsigned period_s;
  bool optional;
};
inline constexpr ProbeSpec kProbeSpecs[] = {
  {0, 0x0300, 0, 20, 3, false},  // ESC error counters, never clear
  {0, 0x092C, 0, 4, 3, false},   // filtered DC difference, NOT PDO arrival phase
  {1, 0x2070, 0, 2, 60, false}, {1, 0x2200, 0, 2, 10, false},
  {1, 0x100A, 0, 64, 300, true},  // optional software version string
  {1, 0x2002, 0, 2, 60, false},  // selector only, not the selected monitor value
  {1, 0x2031, 0, 2, 60, false}, {1, 0x2032, 0, 2, 60, false},
  {1, 0x2033, 0, 2, 60, false}, {1, 0x2034, 0, 2, 60, false},
  {1, 0x2035, 0, 2, 60, false}, {1, 0x2063, 0, 2, 60, false},
  {1, 0x2064, 0, 2, 60, false}, {1, 0x2065, 0, 2, 60, false},
  {1, 0x2066, 0, 2, 60, false}, {1, 0x2067, 0, 2, 60, false},
  {1, 0x2025, 0, 2, 60, false},
  {1, 0x1C32, 1, 2, 60, true}, {1, 0x1C32, 2, 4, 60, true},
  {1, 0x1C32, 0x0B, 2, 30, true}, {1, 0x1C32, 0x0C, 2, 30, true},
  {1, 0x1C32, 0x20, 1, 30, true},
  {1, 0x1C33, 1, 2, 60, true}, {1, 0x1C33, 2, 4, 60, true},
  {1, 0x1C33, 0x0B, 2, 30, true}, {1, 0x1C33, 0x0C, 2, 30, true},
  {1, 0x1C33, 0x20, 1, 30, true},
};
constexpr std::size_t kProbeCount = sizeof(kProbeSpecs) / sizeof(kProbeSpecs[0]);
constexpr std::size_t kProbeJobs = 7 * kProbeCount;
constexpr std::size_t kProbePayload = 13;

// Bounded, allocation-free state machine. One outstanding request per master,
// at most ten starts/s, one state poll/10 ms, no catch-up burst. A stuck request
// is never reused; it pauses probing (not PDO control) until it completes.
// Transport supplies start(slave,spec), state(), size(), data().
class BusProbe
{
public:
  enum {Unused = 0, Busy = 1, Success = 2, Error = 3};
  void reset() {*this = BusProbe{};}

  template<class Transport, class Sink>
  void step(std::uint64_t now, Transport & transport, Sink & sink, bool ready = true)
  {
    if (!initialized_) {
      if (!ready) {return;}
      initialized_ = true;
      next_start_ = now + 5000000000ULL;  // let normal startup settle
    }
    if (now < next_poll_) {return;}
    next_poll_ = now + 10000000ULL;
    if (pending_) {
      const auto state = transport.state();
      if (state == Busy) {
        if (!timeout_reported_ && now - started_ >= 1000000000ULL) {
          emit(now, -5, 0, 0, nullptr, false, sink);
          timeout_reported_ = true;
        }
        return;
      }
      const auto & spec = kProbeSpecs[active_ / 7];
      const auto size = state == Success ? transport.size() : 0;
      const bool valid_size = size > 0 && size <= 64 &&
        (spec.index == 0x100A || size == spec.size);
      const bool ok = state == Success && valid_size;
      finish(now, ok ? 0 : (state == Success ? -2 : -1),
        state, size, ok ? transport.data() : nullptr, sink);
      return;
    }
    if (!ready || now < next_start_) {return;}
    for (std::size_t n = 0; n < kProbeJobs; ++n) {
      const auto job = cursor_;
      cursor_ = (cursor_ + 1) % kProbeJobs;
      if (disabled_[job] || now < due_[job]) {continue;}
      active_ = job;
      started_ = now;
      timeout_reported_ = false;
      next_start_ = now + 100000000ULL;
      const int result = transport.start(job % 7, kProbeSpecs[job / 7]);
      if (result != 0) {finish(now, -3, result, 0, nullptr, sink);}
      else {pending_ = true;}
      return;
    }
  }

private:
  template<class Sink>
  void emit(std::uint64_t now, int outcome, int api_result, std::size_t size,
    const std::uint8_t * bytes, bool disabled, Sink & sink)
  {
    const auto & spec = kProbeSpecs[active_ / 7];
    CycleFrame frame{};
    frame[0] = ++sequence_;
    frame[1] = started_; frame[2] = now; frame[3] = active_ % 7;
    frame[4] = spec.kind; frame[5] = spec.index; frame[6] = spec.sub;
    frame[7] = spec.size; frame[8] = size; frame[9] = outcome;
    frame[10] = api_result; frame[11] = disabled; frame[12] = sink.dropped();
    if (bytes) {
      for (std::size_t i = 0; i < size; ++i) {frame[kProbePayload + i] = bytes[i];}
    }
    sink.push(frame);
  }
  template<class Sink>
  void finish(std::uint64_t now, int outcome, int result, std::size_t size,
    const std::uint8_t * data, Sink & sink)
  {
    const auto & spec = kProbeSpecs[active_ / 7];
    if (outcome == 0) {failures_[active_] = 0;}
    else {
      ++failures_[active_];
      // Unknown/unsupported optional objects fail once, then stay disabled
      // until the next configure. A failed read is never published as zero.
      disabled_[active_] = spec.optional || failures_[active_] >= 3;
    }
    due_[active_] = now + (outcome == 0 ? spec.period_s : 60ULL) * 1000000000ULL;
    emit(now, outcome, result, size, data, disabled_[active_], sink);
    pending_ = false;
  }
  std::array<std::uint64_t, kProbeJobs> due_{};
  std::array<unsigned, kProbeJobs> failures_{};
  std::array<bool, kProbeJobs> disabled_{};
  std::size_t cursor_{0}, active_{0};
  std::uint64_t sequence_{0}, started_{0}, next_start_{0}, next_poll_{0};
  bool initialized_{false}, pending_{false}, timeout_reported_{false};
};
}  // namespace kaixuan_can_hardware
