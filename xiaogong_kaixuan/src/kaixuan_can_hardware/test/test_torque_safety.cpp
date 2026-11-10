#include "kaixuan_can_hardware/torque_safety.hpp"
#include <cstdlib>
#include <iostream>
using namespace kaixuan_can_hardware;
void check(bool condition, const char *message)
{if (!condition) {std::cerr << message << '\n'; std::exit(1);}}
int main()
{
  for (auto control : {0, 6}) {
    check(cia402_disabled_feedback(control, 0x231, 0), "shutdown feedback is torque-disabled");
    check(cia402_disabled_feedback(control, 0x250, 0), "disable-voltage feedback is torque-disabled");
    check(!cia402_disabled_feedback(control, 0x237, 0), "enabled feedback never qualifies as disabled");
    check(!cia402_disabled_feedback(control, 0x250, 1), "drive error prevents disabled qualification");
  }
  for (auto control : {7, 15, 128}) {
    check(!cia402_disabled_feedback(control, 0x250, 0), "enable/fault-reset requests never qualify as disabled");
  }
  TorqueParameters p{15.0, 1000.0 / 15.0, 0.75, 7.5, 0.3, 0.17, -3.14, 3.14};
  check(p.valid(), "valid parameters");
  for (double previous : {-0.75, -0.01, 0.0, 0.01, 0.75}) {
    for (double desired : {-0.75, -0.01, 0.0, 0.01, 0.75}) {
      const double legacy = previous + std::clamp(desired-previous, -0.0075, 0.0075);
      check(std::abs(slew_torque(previous, desired, .001, 7.5, 7.5)-legacy) < 1e-12,
        "default slew exactly preserves legacy reversals");
      const double fast = slew_torque(previous, desired, .001, 7.5, 30.0);
      check(fast >= std::min(previous, desired)-1e-12 &&
        fast <= std::max(previous, desired)+1e-12, "release never overshoots desired torque");
      if (fast * previous >= 0 && std::abs(fast) > std::abs(previous)) {
        check(std::abs(fast-previous) <= .0075+1e-12, "increasing magnitude retains original slew");
      }
    }
  }
  check(std::abs(slew_torque(.5, .1, .001, 7.5, 30.)-.47) < 1e-12,
    "positive torque unloads at release rate");
  check(std::abs(slew_torque(-.5, -.1, .001, 7.5, 30.)+.47) < 1e-12,
    "negative torque unloads at release rate");
  check(std::abs(slew_torque(.015, -.5, .001, 7.5, 30.)+.00375) < 1e-12,
    "reversal only builds opposite torque during remaining half-period");
  p.release_slew_nm_s = -1.; check(!p.valid(), "negative release rate rejected");
  p.release_slew_nm_s = 1.; check(!p.valid(), "release slower than increase rejected");
  p.release_slew_nm_s = 30.; check(p.valid(), "explicit release rate valid");
  p.release_slew_nm_s = 0.;
  check(p.to_raw(1000, 1) == 50 && p.to_raw(-1000, -1) == 50, "clamp/sign/overflow");
  check(std::abs(p.feedback_nm(50, -1, 0, 1) + 0.75) < 1e-12, "per-mille feedback");
  check(std::abs(p.feedback_nm(5, 1, 1, 1) - 0.75) < 1e-12, "current feedback");
  TorqueSafety guard;
  TorqueSafety::Input in;
  in.requested = in.startup_ready = in.bus_ready = in.write_fresh = true;
  in.mode = 10; in.status = 0x23;
  check(!guard.step(in, p).enable, "no command must not enable");
  in.command = 0.1;
  check(!guard.step(in, p).enable, "first nonzero must not enable");
  in.command = 0;
  check(guard.step(in, p).enable, "fresh zero enables");
  in.command = 0.7;
  check(guard.step(in, p).effort_nm == 0, "zero torque until operation enabled");
  in.status = 0x27;
  check(std::abs(guard.step(in, p).effort_nm - 0.0075) < 1e-12, "slew limit");
  in.command = 100;
  for (int i = 0; i < 200; ++i) {guard.step(in, p);}
  check(guard.output_nm == 0.75, "output clamp");
  in.command = std::numeric_limits<double>::quiet_NaN();
  check(!guard.step(in, p).enable && guard.output_nm == 0, "invalid input zeroes/disables");
  in.command = 0;
  check(!guard.step(in, p).enable, "fault stays latched on fresh zero");
  guard.reset();
  check(guard.step(in, p).enable, "claim restart plus zero rearms");
  in.write_fresh = false;
  check(!guard.step(in, p).enable && guard.fault == TorqueSafety::Fault::WriteTimeout, "controller-manager timeout");
  in.write_fresh = true;
  for (int test = 0; test < 6; ++test) {
    guard.reset(); in.mode = 10; in.bus_ready = true; in.error = 0; in.status = 0x27;
    in.position = in.velocity = 0; in.command = 0;
    check(guard.step(in, p).enable, "arm before fault test");
    if (test == 0) {in.mode = 8;}
    if (test == 1) {in.bus_ready = false;}
    if (test == 2) {in.error = 400; in.status = 0x18;}
    if (test == 3) {in.position = 0.2;}
    if (test == 4) {in.velocity = 0.31;}
    if (test == 5) {in.position = std::numeric_limits<double>::infinity();}
    check(!guard.step(in, p).enable && guard.output_nm == 0, "fault must stop torque");
  }
  guard.reset(); p.normalize_position = true;
  p.min_position_rad = -3.141592653589793; p.max_position_rad = 3.141592653589793;
  in.mode = 10; in.bus_ready = in.write_fresh = true; in.error = 0; in.status = 0x27;
  in.command = in.velocity = 0;
  in.position = in.unwrapped_position = -3.141592653589793 + 0.001;
  check(guard.step(in, p).enable, "normalized angle inside exact pi bounds enables");
  in.position = 3.141592653589793 - 0.001;
  in.unwrapped_position = -3.141592653589793 - 0.001;
  check(guard.step(in, p).enable, "crossing pi must not look like a full-turn displacement");
  in.unwrapped_position += 2 * 3.141592653589793;
  check(!guard.step(in, p).enable && guard.fault == TorqueSafety::Fault::Displacement,
    "continuous encoder displacement still detects a full turn");
  // First-fault evidence survives deceleration and later invalid commands.
  guard.reset(); p.normalize_position = false;
  in.position = 0.05; in.velocity = 0; in.command = 0;
  check(guard.step(in, p).enable, "arm snapshot test");
  in.velocity = 0.4;
  check(!guard.step(in, p).enable && guard.fault_snapshot.valid, "capture velocity fault");
  check(std::abs(guard.fault_snapshot.input.velocity - 0.4) < 1e-12 &&
    std::abs(guard.fault_snapshot.anchor - 0.05) < 1e-12, "record triggering speed and enable anchor");
  in.velocity = 0; in.command = std::numeric_limits<double>::quiet_NaN();
  guard.step(in, p);
  check(guard.fault == TorqueSafety::Fault::Velocity &&
    std::abs(guard.fault_snapshot.input.velocity - 0.4) < 1e-12,
    "latched evidence is not overwritten by stopped feedback or command timeout");
  guard.reset(); in.command = 0;
  check(!guard.fault_snapshot.valid && guard.step(in, p).enable, "reset clears old evidence");
  in.position = 0.25;
  check(!guard.step(in, p).enable && guard.fault == TorqueSafety::Fault::Displacement &&
    std::abs(guard.fault_snapshot.displacement - 0.2) < 1e-12,
    "capture displacement relative to the original enable position");
  // User-selected manual range remains bounded at 90 deg/s and 120 degrees.
  p.max_velocity_rad_s = 90 * 3.141592653589793 / 180;
  p.max_displacement_rad = 120 * 3.141592653589793 / 180;
  guard.reset(); in.position = 0; in.velocity = 0;
  check(guard.step(in, p).enable, "arm wide-range test");
  in.position = 119 * 3.141592653589793 / 180;
  check(guard.step(in, p).enable, "119 degree displacement is allowed");
  in.position = 121 * 3.141592653589793 / 180;
  check(!guard.step(in, p).enable && guard.fault == TorqueSafety::Fault::Displacement,
    "121 degree displacement is still rejected");
  for (double speed : {45.013, -45.013, 89.9, -89.9, 90.0, -90.0}) {
    guard.reset(); in.position = in.velocity = 0;
    check(guard.step(in, p).enable, "rearm allowed-speed test");
    in.velocity = speed * 3.141592653589793 / 180;
    check(guard.step(in, p).enable, "speeds up to 90 deg/s are allowed in either direction");
  }
  for (double speed : {90.1, -90.1}) {
    guard.reset(); in.position = in.velocity = 0;
    check(guard.step(in, p).enable, "rearm overspeed test");
    in.velocity = speed * 3.141592653589793 / 180;
    check(!guard.step(in, p).enable && guard.fault == TorqueSafety::Fault::Velocity,
      "speeds above 90 deg/s still stop torque in either direction");
  }
  p.limit_nm = std::numeric_limits<double>::infinity();
  check(!p.valid(), "nonfinite limits rejected");
  std::array<TorqueSafety, 3> group;
  std::array<bool, 3> claimed{true, true, false};
  group[0].trip(TorqueSafety::Fault::Velocity);
  check(latch_torque_group(group, claimed), "one drive fault stops the claimed group");
  check(group[0].fault == TorqueSafety::Fault::Velocity &&
    group[1].fault == TorqueSafety::Fault::Peer && group[2].fault == TorqueSafety::Fault::None,
    "preserve originating fault, latch peers, leave unclaimed joints alone");
  check(!group[1].fault_snapshot.valid, "peer stops must not fabricate triggering measurements");
  group[0].reset();
  check(latch_torque_group(group, claimed) && group[0].fault == TorqueSafety::Fault::Peer,
    "restarting one joint cannot clear a group stop");
  for (auto & member : group) {member.reset();}
  check(!latch_torque_group(group, claimed), "group release and restart clears stop");
  std::cout << "CST scaling, arming, timeout and motion guard checks passed\n";
}
