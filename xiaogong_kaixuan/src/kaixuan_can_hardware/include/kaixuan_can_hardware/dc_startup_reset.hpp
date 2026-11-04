#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace kaixuan_can_hardware
{
// Single cyclic-thread owner. This class performs no IO. The caller owns a
// dedicated 16-byte register request, executes each nonempty action once, and
// echoes its token in subsequent replies. Never change a BUSY request's buffer.
// fresh_disabled means current complete-WKC feedback, sent CW6, all relevant
// axes explicitly ReadyToSwitchOn (0x21) or SwitchOnDisabled (0x40), and error=0.
// Call after IgH has configured offsets; this helper never changes their values.
class DcStartupReset
{
public:
  static constexpr unsigned kDisabledCycles = 20;
  enum class Kind {None, Read, Write};
  enum class RequestState {None, Busy, Success, Error, Rejected};
  enum class Phase {
    Bypass, WaitDisabled, ReadType, ReadProduct, ReadSpeedCounterStart, ReadSpeedFilter,
    WriteSpeedFilter, WriteSpeedCounterStart, VerifySpeedCounterStart, VerifySpeedFilter,
    Observe, Ready, Failed
  };
  enum class Error {
    None, RequestFailed, RequestRejected, RequestTimeout, BadReadSize, WrongType,
    WrongProduct, WrongVendor, VerifyMismatch, InvalidSpeedCounterStart
  };
  struct Action {
    Kind kind{Kind::None};
    std::uint64_t token{0};
    std::uint16_t address{0};
    std::size_t size{0};
    std::array<std::uint8_t, 16> bytes{};
  };
  struct Reply {
    std::uint64_t token{0};
    RequestState state{RequestState::None};
    std::size_t size{0};
    std::array<std::uint8_t, 16> bytes{};
  };
  struct Input {
    std::uint64_t now_ns{0};
    bool fresh_disabled{false};
    Reply reply{};
  };
  struct Status {
    Phase phase{Phase::Bypass};
    bool pending{false}, ready{false}, failed{false};
    Error error{Error::None};
    unsigned disabled_cycles{0};
    bool original_start_valid{false}, original_filter_valid{false};
    std::uint16_t speed_counter_start{0};
    std::uint8_t speed_filter{0};
    std::uint64_t observation_start_ns{0};
  };
  struct Output {
    Action action{};
    Status status{};
    // One-shot after BOTH writebacks have been read back. Reset StartupGuard
    // and discard any DC monitor response queued before these writebacks.
    bool discard_old_dc_samples{false};
  };

  explicit DcStartupReset(bool enabled = false,
    std::uint64_t request_timeout_ns = 1000000000ULL,
    std::uint64_t observation_ns = 10000000000ULL)
  : phase_(enabled ? Phase::WaitDisabled : Phase::Bypass),
    request_timeout_ns_(request_timeout_ns), observation_ns_(observation_ns) {}

  Status status() const {return finish(Output{}).status;}

  Output step(const Input & in)
  {
    Output out;
    // Ready is latched: enabling the drives must not revoke this permission.
    // A normal deactivate/reactivate retains it; rebuild only with the master.
    if (phase_ == Phase::Bypass || phase_ == Phase::Ready) {return finish(out);}
    if (!in.fresh_disabled) {disabled_cycles_ = 0;}
    else if (disabled_cycles_ < kDisabledCycles) {++disabled_cycles_;}

    if (pending()) {
      // Success first observed at/after the deadline is late as well. A
      // timeout does not release ownership of a potentially still BUSY request.
      if (elapsed(in.now_ns, request_started_ns_, request_timeout_ns_)) {
        fail(Error::RequestTimeout);
      }
      const bool terminal = in.reply.token == pending_.token &&
        (in.reply.state == RequestState::Success || in.reply.state == RequestState::Error ||
        in.reply.state == RequestState::Rejected);
      if (!terminal) {return finish(out);}
      const auto operation = pending_phase_;
      pending_ = Action{};
      if (error_ == Error::None) {complete(operation, in.reply, in.now_ns, out);}
    }
    if (error_ != Error::None) {return finish(out);}

    if (phase_ == Phase::Observe) {
      if (disabled_cycles_ >= kDisabledCycles &&
        elapsed(in.now_ns, observation_start_ns_, observation_ns_))
      {
        phase_ = Phase::Ready;
      }
      return finish(out);
    }
    if (disabled_cycles_ < kDisabledCycles) {return finish(out);}
    if (phase_ == Phase::WaitDisabled) {phase_ = Phase::ReadType;}
    switch (phase_) {
      case Phase::ReadType: issue(out, in.now_ns, Kind::Read, 0x0000, 1); break;
      case Phase::ReadProduct: issue(out, in.now_ns, Kind::Read, 0x0e00, 16); break;
      case Phase::ReadSpeedCounterStart:
      case Phase::VerifySpeedCounterStart:
        issue(out, in.now_ns, Kind::Read, 0x0930, 2); break;
      case Phase::ReadSpeedFilter:
      case Phase::VerifySpeedFilter:
        issue(out, in.now_ns, Kind::Read, 0x0935, 1); break;
      case Phase::WriteSpeedFilter:
        issue(out, in.now_ns, Kind::Write, 0x0935, 1, speed_filter_); break;
      case Phase::WriteSpeedCounterStart:
        issue(out, in.now_ns, Kind::Write, 0x0930, 2, speed_counter_start_); break;
      default: break;
    }
    return finish(out);
  }

private:
  bool pending() const {return pending_.kind != Kind::None;}
  static bool elapsed(std::uint64_t now, std::uint64_t start, std::uint64_t duration)
  {
    return now >= start && now - start >= duration;
  }
  static std::uint64_t little_endian(const std::uint8_t * bytes, std::size_t size)
  {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < size; ++i) {value |= std::uint64_t(bytes[i]) << (8 * i);}
    return value;
  }
  void fail(Error error)
  {
    if (error_ == Error::None) {error_ = error;}
    phase_ = Phase::Failed;
  }
  void issue(Output & out, std::uint64_t now, Kind kind, std::uint16_t address,
    std::size_t size, std::uint16_t value = 0)
  {
    pending_ = Action{};
    pending_.kind = kind;
    pending_.token = ++sequence_;
    pending_.address = address;
    pending_.size = size;
    pending_.bytes[0] = static_cast<std::uint8_t>(value);
    pending_.bytes[1] = static_cast<std::uint8_t>(value >> 8);
    pending_phase_ = phase_;
    request_started_ns_ = now;
    out.action = pending_;
  }
  void complete(Phase operation, const Reply & reply, std::uint64_t now, Output & out)
  {
    if (reply.state != RequestState::Success) {
      fail(reply.state == RequestState::Rejected ? Error::RequestRejected : Error::RequestFailed);
      return;
    }
    if (operation == Phase::WriteSpeedFilter) {phase_ = Phase::WriteSpeedCounterStart; return;}
    if (operation == Phase::WriteSpeedCounterStart) {phase_ = Phase::VerifySpeedCounterStart; return;}
    const std::size_t expected = operation == Phase::ReadProduct ? 16 :
      (operation == Phase::ReadSpeedCounterStart || operation == Phase::VerifySpeedCounterStart ? 2 : 1);
    if (reply.size != expected) {fail(Error::BadReadSize); return;}
    switch (operation) {
      case Phase::ReadType:
        if (reply.bytes[0] != 0xc0) {fail(Error::WrongType);}
        else {phase_ = Phase::ReadProduct;}
        break;
      case Phase::ReadProduct:
        if (little_endian(reply.bytes.data() + 2, 2) != 0x9253) {fail(Error::WrongProduct);}
        else if (little_endian(reply.bytes.data() + 8, 8) != 0x04d8) {fail(Error::WrongVendor);}
        else {phase_ = Phase::ReadSpeedCounterStart;}
        break;
      case Phase::ReadSpeedCounterStart:
        speed_counter_start_ = static_cast<std::uint16_t>(little_endian(reply.bytes.data(), 2));
        original_start_valid_ = true;
        if (speed_counter_start_ < 0x0080 || speed_counter_start_ > 0x3fff) {
          fail(Error::InvalidSpeedCounterStart);
        } else {phase_ = Phase::ReadSpeedFilter;}
        break;
      case Phase::ReadSpeedFilter:
        speed_filter_ = reply.bytes[0];
        original_filter_valid_ = true;
        phase_ = Phase::WriteSpeedFilter;
        break;
      case Phase::VerifySpeedCounterStart:
        if (little_endian(reply.bytes.data(), 2) != speed_counter_start_) {fail(Error::VerifyMismatch);}
        else {phase_ = Phase::VerifySpeedFilter;}
        break;
      case Phase::VerifySpeedFilter:
        if (reply.bytes[0] != speed_filter_) {fail(Error::VerifyMismatch);}
        else {
          phase_ = Phase::Observe;
          observation_start_ns_ = now;
          out.discard_old_dc_samples = true;
        }
        break;
      default: break;
    }
  }
  Output finish(Output out) const
  {
    out.status.phase = phase_;
    out.status.pending = pending();
    out.status.ready = phase_ == Phase::Bypass || phase_ == Phase::Ready;
    out.status.failed = error_ != Error::None;
    out.status.error = error_;
    out.status.disabled_cycles = disabled_cycles_;
    out.status.original_start_valid = original_start_valid_;
    out.status.original_filter_valid = original_filter_valid_;
    out.status.speed_counter_start = speed_counter_start_;
    out.status.speed_filter = speed_filter_;
    out.status.observation_start_ns = observation_start_ns_;
    return out;
  }

  Phase phase_, pending_phase_{Phase::Bypass};
  Error error_{Error::None};
  Action pending_{};
  std::uint64_t request_timeout_ns_, observation_ns_;
  std::uint64_t sequence_{0}, request_started_ns_{0}, observation_start_ns_{0};
  unsigned disabled_cycles_{0};
  bool original_start_valid_{false}, original_filter_valid_{false};
  std::uint16_t speed_counter_start_{0};
  std::uint8_t speed_filter_{0};
};
}  // namespace kaixuan_can_hardware
