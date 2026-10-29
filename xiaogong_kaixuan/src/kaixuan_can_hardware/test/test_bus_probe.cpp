#include "kaixuan_can_hardware/bus_probe.hpp"
#include <iostream>
#include <map>
#include <stdexcept>
#include <tuple>
#include <vector>

using namespace kaixuan_can_hardware;
void check(bool ok) {if (!ok) {throw std::runtime_error("bus probe test failed");}}
struct FakeTransport
{
  std::uint64_t now{0};
  int response{BusProbe::Success}, start_result{0};
  bool wrong_size{false};
  ProbeSpec spec{};
  std::array<std::uint8_t, 64> bytes{};
  std::vector<std::uint64_t> starts;
  std::map<std::tuple<std::size_t, unsigned, unsigned>, unsigned> jobs;
  int start(std::size_t slave, const ProbeSpec & job)
  {
    spec = job; starts.push_back(now); ++jobs[{slave, job.index, job.sub}];
    return start_result;
  }
  int state() {return response;}
  std::size_t size() {return wrong_size ? 65 : spec.size;}
  const std::uint8_t * data() {return bytes.data();}
};
struct Sink
{
  std::vector<CycleFrame> frames;
  unsigned dropped() const {return 0;}
  void push(const CycleFrame & frame) {frames.push_back(frame);}
};

int main()
{
  BusProbe probe;
  FakeTransport transport;
  Sink sink;
  // No probing while startup has no complete/OP bus. Pending reads, however,
  // still progress if the bus loses readiness later.
  probe.step(1000000000ULL, transport, sink, false);
  check(transport.starts.empty());
  auto tick = [&](std::uint64_t ms) {
      transport.now = ms * 1000000ULL;
      probe.step(transport.now, transport, sink);
    };
  tick(0); tick(4999); check(transport.starts.empty());
  tick(5010); check(transport.starts.size() == 1);
  tick(5020); check(sink.frames.size() == 1 && sink.frames[0][9] == 0);
  check(sink.frames[0][5] == 0x0300 && sink.frames[0][8] == 20);
  for (unsigned ms = 5030; ms < 120000; ms += 10) {tick(ms);}
  check(transport.jobs.size() == kProbeJobs);
  for (std::size_t i = 1; i < transport.starts.size(); ++i) {
    check(transport.starts[i] - transport.starts[i-1] >= 100000000ULL);
  }
  for (std::size_t slave = 0; slave < 7; ++slave) {
    check(transport.jobs[{slave, 0x0300, 0}] > 10);
    check(transport.jobs[{slave, 0x2070, 0}] >= 2);
  }

  // A stuck request emits one timeout evidence record and does not overlap
  // another job. Recovery resumes only after that same request completes.
  probe.reset(); transport = FakeTransport{}; sink.frames.clear();
  transport.response = BusProbe::Busy;
  tick(0);
  for (unsigned ms = 5000; ms < 10000; ms += 10) {tick(ms);}
  check(transport.starts.size() == 1 && sink.frames.size() == 1);
  check(sink.frames[0][9] == -5);
  transport.response = BusProbe::Success;
  probe.step(10000000000ULL, transport, sink, false);
  tick(10010);
  check(sink.frames.size() == 2 && sink.frames.back()[9] == 0);
  check(transport.starts.size() == 2);

  // Optional objects disable after the first failed read; documented objects
  // retry no more than three times, at least 60 s apart, then disable too.
  probe.reset(); transport = FakeTransport{}; sink.frames.clear();
  transport.response = BusProbe::Error;
  tick(0);
  for (unsigned ms = 5000; ms < 300000; ms += 10) {tick(ms);}
  check(transport.jobs[{0, 0x100A, 0}] == 1);
  check(transport.jobs[{0, 0x2070, 0}] == 3);
  check(transport.jobs[{6, 0x1C33, 0x20}] == 1);
  for (const auto & frame : sink.frames) {
    check(frame[9] == -1 && frame[8] == 0);
  }

  probe.reset(); transport = FakeTransport{}; sink.frames.clear();
  transport.wrong_size = true;
  tick(0); tick(5000); tick(5010);
  check(sink.frames.size() == 1 && sink.frames[0][9] == -2);
  check(sink.frames[0][8] == 65);  // no payload overflow
  probe.reset(); transport = FakeTransport{}; sink.frames.clear();
  transport.start_result = -19;
  tick(0); tick(5000);
  check(sink.frames.size() == 1 && sink.frames[0][9] == -3 && sink.frames[0][10] == -19);
  std::cout << "PASS: rate limit, fairness, stuck request, backoff, disable, size, start failure\n";
}
