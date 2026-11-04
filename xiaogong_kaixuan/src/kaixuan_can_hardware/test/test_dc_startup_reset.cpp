#include "kaixuan_can_hardware/dc_startup_reset.hpp"
#include "kaixuan_can_hardware/startup_guard.hpp"

#include <cstdlib>
#include <iostream>
#include <vector>

namespace
{
using Reset = kaixuan_can_hardware::DcStartupReset;
using Kind = Reset::Kind;
using State = Reset::RequestState;
using Phase = Reset::Phase;
using Error = Reset::Error;
int checks = 0;
void check(bool condition, const char * expression, int line)
{
  ++checks;
  if (!condition) {
    std::cerr << "FAIL line " << line << ": " << expression << '\n';
    std::exit(1);
  }
}
#define CHECK(expression) check(bool(expression), #expression, __LINE__)

// Memory-only transport. A write takes effect on submission, so a failed reply
// cannot be treated as proof that a write did not reach the device.
struct Fixture
{
  explicit Fixture(bool enabled = true) : reset(enabled)
  {
    product[2] = 0x53; product[3] = 0x92;
    product[8] = 0xd8; product[9] = 0x04;
  }
  Reset reset;
  std::uint64_t now{0};
  bool disabled{true};
  std::uint8_t type{0xc0}, filter{0xac};
  std::uint16_t start{0x1000};
  std::array<std::uint8_t, 16> product{};
  Reset::Action active{};
  Reset::Output result{};
  std::vector<Reset::Action> actions;
  unsigned discard_events{0};

  Reset::Output tick(Reset::Reply reply = {}, std::uint64_t advance = 1)
  {
    now += advance;
    const bool terminal = reply.token == active.token &&
      (reply.state == State::Success || reply.state == State::Error || reply.state == State::Rejected);
    if (terminal) {active = {};}
    result = reset.step({now, disabled, reply});
    discard_events += result.discard_old_dc_samples;
    if (result.action.kind != Kind::None) {
      CHECK(active.kind == Kind::None);  // No live request may be replaced.
      CHECK(disabled && result.status.disabled_cycles == Reset::kDisabledCycles);
      active = result.action;
      actions.push_back(active);
      if (active.kind == Kind::Write) {
        CHECK(result.status.original_start_valid && result.status.original_filter_valid);
        if (active.address == 0x0935) {
          CHECK(active.size == 1 && active.bytes[0] == result.status.speed_filter);
          filter = active.bytes[0];
        } else {
          CHECK(active.address == 0x0930 && active.size == 2);
          start = std::uint16_t(active.bytes[0]) | (std::uint16_t(active.bytes[1]) << 8);
          CHECK(start == result.status.speed_counter_start);
        }
      }
    }
    CHECK(!result.status.ready || !result.status.failed);
    CHECK(result.status.pending == (active.kind != Kind::None));
    return result;
  }
  Reset::Reply response(State state = State::Success) const
  {
    CHECK(active.kind != Kind::None);
    Reset::Reply reply;
    reply.token = active.token; reply.state = state; reply.size = active.size;
    if (active.kind == Kind::Read) {
      if (active.address == 0x0000) {reply.bytes[0] = type;}
      else if (active.address == 0x0e00) {reply.bytes = product;}
      else if (active.address == 0x0930) {
        reply.bytes[0] = std::uint8_t(start); reply.bytes[1] = std::uint8_t(start >> 8);
      } else {CHECK(active.address == 0x0935); reply.bytes[0] = filter;}
    }
    return reply;
  }
  void complete(State state = State::Success, std::uint64_t advance = 1)
  {
    tick(response(state), advance);
  }
  void qualify()
  {
    for (unsigned i = 0; i < Reset::kDisabledCycles; ++i) {tick();}
  }
  void to_action(unsigned index)
  {
    qualify();
    for (unsigned i = 0; i < index; ++i) {complete();}
    CHECK(active.kind != Kind::None && actions.size() == index + 1);
  }
  void to_observe()
  {
    to_action(7); complete();
    CHECK(result.status.phase == Phase::Observe && !result.status.ready);
    CHECK(result.discard_old_dc_samples && discard_events == 1);
  }
};

void bypass_and_qualification()
{
  Reset bypass;
  CHECK(bypass.status().ready && bypass.status().phase == Phase::Bypass);
  CHECK(!Reset(true).status().ready);
  Fixture f(false);
  f.disabled = false;
  for (int i = 0; i < 30; ++i) {
    CHECK(f.tick().status.ready);
    CHECK(!f.result.discard_old_dc_samples);
  }
  CHECK(f.actions.empty());
  Fixture enabled;
  for (int i = 0; i < 19; ++i) {enabled.tick();}
  CHECK(enabled.actions.empty() && !enabled.result.status.ready);
  enabled.disabled = false; enabled.tick();
  CHECK(enabled.result.status.disabled_cycles == 0);
  enabled.disabled = true;
  for (int i = 0; i < 19; ++i) {enabled.tick();}
  CHECK(enabled.actions.empty());
  enabled.tick();
  CHECK(enabled.active.address == 0 && enabled.active.size == 1);
}

void exact_writebacks_observation_and_latching()
{
  Fixture f;
  f.start = 0x1234;  // Distinguish both bytes, including byte order.
  f.to_observe();
  CHECK(f.actions.size() == 8);
  const std::uint16_t addresses[] = {0, 0x0e00, 0x0930, 0x0935, 0x0935, 0x0930, 0x0930, 0x0935};
  for (unsigned i = 0; i < 8; ++i) {
    CHECK(f.actions[i].address == addresses[i]);
    CHECK(f.actions[i].kind == (i == 4 || i == 5 ? Kind::Write : Kind::Read));
    CHECK(f.actions[i].token == i + 1);
  }
  CHECK(f.actions[4].bytes[0] == 0xac);  // Full byte, high nibble unchanged.
  CHECK(f.actions[5].bytes[0] == 0x34 && f.actions[5].bytes[1] == 0x12);
  CHECK(f.start == 0x1234 && f.filter == 0xac);
  f.tick({}, 9999999999ULL);
  CHECK(!f.result.status.ready && !f.result.discard_old_dc_samples);
  f.tick();
  CHECK(f.result.status.ready && f.result.status.phase == Phase::Ready);
  f.disabled = false;  // Subsequent drive enable must not revoke readiness.
  for (int i = 0; i < 30; ++i) {CHECK(f.tick().status.ready);}
  f.disabled = true; f.tick();
  CHECK(f.actions.size() == 8 && f.discard_events == 1);
}

void observation_alone_does_not_replace_startup_guard()
{
  Fixture f;
  f.to_observe();
  kaixuan_can_hardware::StartupGuard guard;
  for (int i = 0; i < 200; ++i) {guard.step(true, true, true, i % 20 == 0, 0);}
  CHECK(guard.latched);  // Old zero samples are insufficient evidence after reset.
  if (f.result.discard_old_dc_samples) {guard.reset();}
  f.tick({}, 10000000000ULL);
  CHECK(f.result.status.ready && !guard.latched);
  CHECK(!guard.step(true, true, true, false, 0));
  for (int i = 0; i < 150; ++i) {guard.step(true, true, true, i % 20 == 0, 0);}
  CHECK(guard.latched);
}

void invalid_identity_and_original()
{
  for (int failure = 0; failure < 5; ++failure) {
    Fixture f;
    if (failure == 0) {f.type = 0x11;}
    if (failure == 1) {f.product[2] = 0x52;}
    if (failure == 2) {f.product[8] = 0xd9;}
    if (failure == 3) {f.start = 0x007f;}
    if (failure == 4) {f.start = 0x8000;}
    f.qualify();
    while (f.active.kind != Kind::None) {f.complete();}
    CHECK(f.result.status.failed && !f.result.status.ready);
    const Error expected[] = {Error::WrongType, Error::WrongProduct, Error::WrongVendor,
      Error::InvalidSpeedCounterStart, Error::InvalidSpeedCounterStart};
    CHECK(f.result.status.error == expected[failure]);
    for (const auto & action : f.actions) {CHECK(action.kind == Kind::Read);}
  }
}

void transport_failure_at_every_stage()
{
  for (unsigned index = 0; index < 8; ++index) {
    for (State failure : {State::Error, State::Rejected}) {
      Fixture f;
      f.to_action(index);
      f.complete(failure);
      CHECK(f.result.status.failed && !f.result.status.ready && !f.result.status.pending);
      CHECK(f.result.status.error == (failure == State::Error ? Error::RequestFailed : Error::RequestRejected));
      for (int i = 0; i < 30; ++i) {f.tick({}, 1000000000ULL);}
      CHECK(f.actions.size() == index + 1 && f.discard_events == 0);
      CHECK(f.start == 0x1000 && f.filter == 0xac);  // Never install different parameters.
    }
  }
}

void timeout_busy_late_success_and_old_tokens()
{
  for (unsigned index = 0; index < 8; ++index) {
    Fixture f;
    f.to_action(index);
    const auto saved = f.active;
    auto stale = f.response(); stale.token += 100;
    f.tick(stale);
    CHECK(f.active.token == saved.token && f.actions.size() == index + 1);
    f.tick(f.response(State::Busy), 1000000000ULL);
    CHECK(f.result.status.failed && f.result.status.pending);
    CHECK(f.result.status.error == Error::RequestTimeout);
    CHECK(f.active.token == saved.token && f.active.bytes == saved.bytes);
    f.tick({}, 1000000000ULL);
    CHECK(f.result.status.pending && f.actions.size() == index + 1);
    f.complete();
    CHECK(!f.result.status.pending && f.result.status.failed && !f.result.status.ready);
    CHECK(f.discard_events == 0 && f.actions.size() == index + 1);
  }
  Fixture terminal_late;
  terminal_late.to_action(7);
  terminal_late.complete(State::Success, 1000000000ULL);
  CHECK(terminal_late.result.status.error == Error::RequestTimeout);
  CHECK(!terminal_late.result.status.ready && terminal_late.discard_events == 0);
}

void wrong_lengths_and_readback_mismatch()
{
  for (unsigned index : {0U, 1U, 2U, 3U, 6U, 7U}) {
    Fixture f;
    f.to_action(index);
    auto reply = f.response(); --reply.size;
    f.tick(reply);
    CHECK(f.result.status.error == Error::BadReadSize && !f.result.status.ready);
  }
  for (unsigned index : {6U, 7U}) {
    Fixture f;
    f.to_action(index);
    auto reply = f.response(); reply.bytes[0] ^= 1;
    f.tick(reply);
    CHECK(f.result.status.error == Error::VerifyMismatch && f.discard_events == 0);
  }
}

void disabled_loss_prevents_each_write_and_final_ready()
{
  for (unsigned preceding : {3U, 4U}) {
    Fixture f;
    f.to_action(preceding);
    f.disabled = false; f.complete();
    CHECK(f.active.kind == Kind::None && !f.result.status.ready);
    CHECK(f.actions.size() == preceding + 1);
    f.disabled = true;
    for (int i = 0; i < 19; ++i) {f.tick();}
    CHECK(f.active.kind == Kind::None);
    f.tick();
    CHECK(f.active.kind == Kind::Write && f.actions.size() == preceding + 2);
  }
  Fixture f;
  f.to_observe();
  f.disabled = false; f.tick({}, 10000000000ULL);
  CHECK(!f.result.status.ready);
  f.disabled = true;
  for (int i = 0; i < 19; ++i) {f.tick();}
  CHECK(!f.result.status.ready);
  CHECK(f.tick().status.ready && f.actions.size() == 8 && f.discard_events == 1);
}
}  // namespace

int main()
{
  bypass_and_qualification();
  exact_writebacks_observation_and_latching();
  observation_alone_does_not_replace_startup_guard();
  invalid_identity_and_original();
  transport_failure_at_every_stage();
  timeout_busy_late_success_and_old_tokens();
  wrong_lengths_and_readback_mismatch();
  disabled_loss_prevents_each_write_and_final_ready();
  std::cout << "PASS dc_startup_reset: 8 scenario groups, " << checks << " checks; no hardware IO\n";
}
