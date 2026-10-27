#include "kaixuan_can_hardware/cycle_capture.hpp"
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace kaixuan_can_hardware;
void check(bool condition) {if (!condition) {throw std::runtime_error("capture test failed");}}

int main()
{
  CycleCapture<2> small;
  CycleFrame frame{};
  check(!small.pop(frame));
  frame[0] = 1;
  check(small.push(frame));
  frame[0] = 2;
  check(small.push(frame));
  check(!small.push(frame));
  check(small.dropped() == 1);
  check(small.pop(frame) && frame[0] == 1);
  check(small.pop(frame) && frame[0] == 2);
  small.reset();
  check(small.dropped() == 0 && !small.pop(frame));

  // One-cycle error and reset edge must both survive, while target-reached
  // chatter is still present in raw data but does not trigger event storage.
  check(capture_transition(0x27, 0x08, 0x0f, 0x80, 0, 400, false) == 7);
  check(capture_transition(0x08, 0x40, 0x80, 0x06, 400, 0, false) == 7);
  check(capture_transition(0x27, 0x427, 0x0f, 0x0f, 0, 0, false) == 0);
  check(capture_transition(0x27, 0x27, 0x0f, 0x0f, 0, 0, true) == 8);

  CycleCapture<128> concurrent;
  constexpr std::int64_t count = 100000;
  std::thread producer([&]() {
      CycleFrame input;
      for (std::int64_t n = 1; n <= count; ++n) {
        input.fill(n);
        while (!concurrent.push(input)) {std::this_thread::yield();}
      }
    });
  for (std::int64_t n = 1; n <= count; ++n) {
    while (!concurrent.pop(frame)) {std::this_thread::yield();}
    for (const auto value : frame) {check(value == n);}
  }
  producer.join();
  std::cout << "PASS: ring overflow, reset, one-cycle transitions, 100000 concurrent frames\n";
}
