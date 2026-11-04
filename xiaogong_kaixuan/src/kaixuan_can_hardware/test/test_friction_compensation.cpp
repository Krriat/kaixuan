#include "kaixuan_can_hardware/friction_compensation.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace kaixuan_can_hardware;
void check(bool value, const char * message)
{if (!value) {std::cerr << message << '\n'; std::exit(1);}}
int main()
{
  constexpr double rad = 3.141592653589793 / 180;
  FrictionCompensation friction;
  auto & p = friction.parameters;
  p.model = "coulomb_viscous"; p.coulomb_nm = 1.8; p.viscous_nm_s_per_rad = 14;
  p.min_speed_rad_s = .5*rad; p.max_speed_rad_s = 2*rad;
  p.limit_nm = .6; p.slew_nm_s = 1.2; p.validate(8);
  const auto step = [&](double velocity, double ramp = 1.0, double scale = .25) {
      return friction.update(velocity, .001, ramp, scale, .2*rad, .5*rad, .02);
    };
  check(step(0) == 0 && step(.19*rad) == 0 && step(-.19*rad) == 0, "rest and noise deadband must add no torque");
  double previous = 0, output = 0;
  for (int i = 0; i < 1500; ++i) {
    output = step(rad);
    check(output >= 0 && output <= .6 && std::abs(output-previous) <= .0012+1e-12, "positive assistance respects cap and slew");
    previous = output;
  }
  check(std::abs(output - .25*(1.8+14*rad)) < 1e-9, "identified Coulomb and viscous coefficients used");
  for (int i = 0; i < 1000; ++i) {output = step(90*rad);}
  check(std::abs(output - .25*(1.8+14*2*rad)) < 1e-9, "90deg/s must use measured maximum speed, not extrapolate slope");
  check(step(-rad) == 0, "reversal removes old-direction torque immediately");
  for (int i = 0; i < 1500; ++i) {output = step(-rad); check(output <= 0, "no torque opposing reversed velocity");}
  check(std::abs(output + .25*(1.8+14*rad)) < 1e-9, "negative assistance mirrors the model");
  check(step(0) == 0 && step(0) == 0, "stopping cannot leave residual assistance");
  check(step(rad, 0) == 0 && step(rad, 1, 0) == 0, "startup ramp and scale zero disable assistance");
  for (int i = 0; i < 1500; ++i) {output = step(rad, .4);}
  check(std::abs(output - .4*.25*(1.8+14*rad)) < 1e-9, "startup ramp scales assistance");
  friction.track_applied(0);
  check(step(rad) <= .0012+1e-12, "returning effort headroom cannot bypass slew from actually applied output");
  friction.reset();
  check(step(.200001*rad) < 1e-8, "deadband edge is smooth");
  p.model = "velocity_lookup"; p.speeds = {.5*rad, rad, 2*rad, 4*rad};
  p.torques = {4.18,3.48,3.36,3.36}; p.max_speed_rad_s = 4*rad; p.limit_nm = 1.1;
  p.validate(13.6);
  check(std::abs(p.magnitude(.75*rad)-3.83) < 1e-12, "lookup interpolation");
  check(p.magnitude(0) == 4.18 && p.magnitude(90*rad) == 3.36, "lookup endpoint clamping");
  bool rejected = false;
  p.speeds[1] = p.speeds[0];
  try {p.validate(13.6);} catch (const std::invalid_argument &) {rejected = true;}
  check(rejected, "repeated/unsorted lookup knots rejected");
  rejected = false;
  try {step(std::numeric_limits<double>::quiet_NaN());} catch (const std::invalid_argument &) {rejected = true;}
  check(rejected, "invalid velocity rejected");
  p = FrictionParameters{};
  check(step(rad) == 0, "unidentified/disabled joint receives no friction torque");
  std::cout << "Friction models, low-speed smoothing, bounds, stop/reversal and slew checks passed\n";
}
