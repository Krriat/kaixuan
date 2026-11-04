#include "kaixuan_can_hardware/gravity_model.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
using namespace kaixuan_can_hardware;
void check(bool ok, const char * message)
{if (!ok) {std::cerr << message << '\n'; std::exit(1);}}
const std::string pendulum = R"(
<robot name="pendulum"><link name="base"/><link name="arm">
<inertial><origin xyz="0.5 0 0"/><mass value="2"/>
<inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
<axis xyz="0 1 0"/><limit lower="-3.14" upper="3.14" effort="100" velocity="2"/></joint>
<link name="tool"/><joint name="tool" type="fixed"><parent link="arm"/><child link="tool"/>
<origin xyz="1 0 0"/></joint></robot>)";
int main()
{
  GravityModel model;
  model.configure(pendulum, {0, 0, -9.80665});
  check(model.joints() == std::vector<std::string>{"hinge"}, "named model joint order");
  check(std::abs(model.compute({0})[0] + 9.80665) < 1e-10, "horizontal link compensating torque sign and magnitude");
  check(std::abs(model.compute({1.5707963267948966})[0]) < 1e-10, "vertical pendulum torque is zero");
  model.configure(pendulum, {0, 0, 9.80665});
  check(std::abs(model.compute({0})[0] - 9.80665) < 1e-10, "inverted mounting reverses compensation");
  model.configure(pendulum, {0, 0, -9.80665}, "tool", 1.0, {0.1, 0, 0});
  check(std::abs(model.compute({0})[0] + 2.1 * 9.80665) < 1e-10, "payload mass and COM at the correct tool frame");
  bool rejected = false;
  try {model.compute({NAN});} catch (...) {rejected = true;}
  check(rejected, "nonfinite feedback rejected");
  rejected = false;
  try {model.configure(pendulum, {0, 0, 0});} catch (...) {rejected = true;}
  check(rejected, "invalid gravity configuration rejected");
  std::cout << "Analytical gravity sign, mass, mounting, payload and invalid-state checks passed\n";
}
