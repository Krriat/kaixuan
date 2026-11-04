#include "kaixuan_can_hardware/startup_guard.hpp"
#include <cstdlib>
#include <iostream>
using namespace kaixuan_can_hardware;
void check(bool condition) {if (!condition) {std::cerr << "Startup guard check failed\n"; std::exit(1);}}
int main()
{
  StartupGuard g;
  for (int n = 0; n < 200; ++n) check(!g.step(false, true, true, true, 0));
  for (int n = 0; n < 200; ++n) check(!g.step(true, false, true, true, 0));
  for (int n = 0; n < 200; ++n) check(!g.step(true, true, false, true, 0));
  for (int n = 0; n < 200; ++n) check(!g.step(true, true, true, true, UINT32_MAX));
  check(!g.step(true, true, true, true, 50001));
  g.reset();
  for (int n = 1; n <= 99; ++n) check(!g.step(true, true, true, n % 20 == 0, 1000));
  check(g.step(true, true, true, true, 1000));
  // After qualified enabling, a single WKC fault must not remove arm torque.
  check(g.step(true, true, false, true, UINT32_MAX));
  check(!g.step(false, true, true, true, 0));
  check(!g.latched);
  // One failed DC sample clears the consecutive-good streak.
  for (int n = 0; n < 100; ++n) g.step(true, true, true, false, 0);
  for (int n = 0; n < 4; ++n) check(!g.step(true, true, true, true, 0));
  check(!g.step(true, true, true, true, UINT32_MAX));
  for (int n = 0; n < 4; ++n) check(!g.step(true, true, true, true, 0));
  check(g.step(true, true, true, true, 0));
  // Loss of bus readiness before enabling restarts qualification.
  g.reset();
  for (int n = 0; n < 99; ++n) check(!g.step(true, true, true, true, 0));
  check(!g.step(true, true, false, false, 0));
  for (int n = 0; n < 99; ++n) check(!g.step(true, true, true, true, 0));
  check(g.step(true, true, true, true, 0));
  EnableSettle settle;
  for (int n = 0; n < 100; ++n) check(settle.switched_on_controlword() == 7);
  check(settle.switched_on_controlword() == 15);
  settle.reset();
  check(settle.switched_on_controlword() == 7);
  std::cout << "startup guard and CSP settle checks passed\n";
}
