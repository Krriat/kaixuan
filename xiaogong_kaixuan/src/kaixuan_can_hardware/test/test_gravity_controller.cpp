#include "kaixuan_can_hardware/gravity_controller.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
void check(bool ok, const char * message)
{if (!ok) {std::cerr << message << '\n'; std::exit(1);}}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  kaixuan_can_hardware::GravityController controller;
  check(controller.init("test_gravity_controller") == controller_interface::return_type::OK, "init");
  const std::string robot = R"(<robot name="two_links"><link name="base"/>
    <link name="a"><inertial><origin xyz="0.5 0 0"/><mass value="2"/>
    <inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
    <link name="b"><inertial><origin xyz="0.2 0 0"/><mass value="1"/>
    <inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
    <joint name="joint_a" type="revolute"><parent link="base"/><child link="a"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="20" velocity="2"/></joint>
    <joint name="joint_b" type="revolute"><parent link="base"/><child link="b"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="20" velocity="2"/></joint></robot>)";
  auto node = controller.get_node();
  node->set_parameters({rclcpp::Parameter("joints", std::vector<std::string>{"joint_b", "joint_a"}),
    rclcpp::Parameter("robot_description", robot),
    rclcpp::Parameter("damping", std::vector<double>{0.4, 0.3}),
    rclcpp::Parameter("effort_limits", std::vector<double>{10, 10}),
    rclcpp::Parameter("gravity_scale", 0.5), rclcpp::Parameter("zero_hold_duration", 0.2),
    rclcpp::Parameter("ramp_duration", 1.0)});
  const rclcpp_lifecycle::State state;
  check(controller.on_configure(state) == controller_interface::CallbackReturn::SUCCESS, "configure");
  double a = 10, b = 20, qa = 0, qb = 0, va = .1, vb = .2;
  hardware_interface::CommandInterface ca("joint_a", "effort", &a), cb("joint_b", "effort", &b);
  hardware_interface::StateInterface sa("joint_a", "position", &qa), sb("joint_b", "position", &qb),
    sva("joint_a", "velocity", &va), svb("joint_b", "velocity", &vb);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(ca); commands.emplace_back(cb);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(svb); states.emplace_back(sa); states.emplace_back(sb); states.emplace_back(sva);
  controller.assign_interfaces(std::move(commands), std::move(states));
  const auto update = [&]() {return controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.001));};
  check(controller.on_activate(state) == controller_interface::CallbackReturn::SUCCESS, "activate");
  check(std::isnan(a) && std::isnan(b), "old commands invalidated before startup");
  check(update() == controller_interface::return_type::OK && a == 0 && b == 0, "fresh zero startup despite nonzero gravity");
  auto end = std::chrono::steady_clock::now() + 1250ms;
  while (std::chrono::steady_clock::now() < end) {
    check(update() == controller_interface::return_type::OK, "ramp updates");
    std::this_thread::sleep_for(5ms);
  }
  check(update() == controller_interface::return_type::OK, "full compensation update");
  check(std::abs(a - (-.5 * 9.80665 - .3 * va)) < 1e-8, "model/command/state name mapping and damping for joint_a");
  check(std::abs(b - (-.5 * .2 * 9.80665 - .4 * vb)) < 1e-8, "model/command/state name mapping and damping for joint_b");
  qb = NAN;
  check(update() == controller_interface::return_type::ERROR && std::isnan(a) && std::isnan(b), "bad feedback stops every joint");
  qb = 0; controller.on_deactivate(state); controller.on_activate(state);
  check(update() == controller_interface::return_type::OK && a == 0 && b == 0, "restart begins from new zero");
  va = 100;
  check(update() == controller_interface::return_type::ERROR && std::isnan(a) && std::isnan(b), "effort overflow stops every joint without partial update");
  va = 0; controller.on_deactivate(state); controller.on_activate(state);
  std::this_thread::sleep_for(60ms);
  check(update() == controller_interface::return_type::ERROR && std::isnan(a) && std::isnan(b), "update watchdog stops every joint");
  controller.on_deactivate(state); controller.release_interfaces();
  va = .1; vb = .2;
  node->set_parameters({rclcpp::Parameter("friction_enabled", true),
    rclcpp::Parameter("friction_scale", .25),
    rclcpp::Parameter("friction_velocity_deadband_rad_s", .01),
    rclcpp::Parameter("friction_velocity_full_rad_s", .02),
    rclcpp::Parameter("friction_filter_time_constant_s", 0.0),
    rclcpp::Parameter("friction_models", std::vector<std::string>{"coulomb_viscous", "coulomb_viscous"}),
    rclcpp::Parameter("friction_coulomb_nm", std::vector<double>{.2, .4}),
    rclcpp::Parameter("friction_viscous_nm_s_per_rad", std::vector<double>{0, 0}),
    rclcpp::Parameter("friction_min_speed_rad_s", std::vector<double>{.02, .02}),
    rclcpp::Parameter("friction_max_speed_rad_s", std::vector<double>{.05, .05}),
    rclcpp::Parameter("friction_effort_limits_nm", std::vector<double>{.1, .2}),
    rclcpp::Parameter("friction_slew_nm_s", std::vector<double>{10, 10}),
    rclcpp::Parameter("friction_lookup_offsets", std::vector<std::int64_t>{0, 0, 0}),
    rclcpp::Parameter("effort_limits", std::vector<double>{10, 4.95})});
  check(controller.on_configure(state) == controller_interface::CallbackReturn::SUCCESS, "configure friction");
  std::vector<hardware_interface::LoanedCommandInterface> fc;
  fc.emplace_back(ca); fc.emplace_back(cb);
  std::vector<hardware_interface::LoanedStateInterface> fs;
  fs.emplace_back(svb); fs.emplace_back(sa); fs.emplace_back(sb); fs.emplace_back(sva);
  controller.assign_interfaces(std::move(fc), std::move(fs));
  check(controller.on_activate(state) == controller_interface::CallbackReturn::SUCCESS, "activate friction");
  check(update() == controller_interface::return_type::OK && a == 0 && b == 0, "friction cannot defeat zero preparation");
  end = std::chrono::steady_clock::now() + 1300ms;
  while (std::chrono::steady_clock::now() < end) {check(update() == controller_interface::return_type::OK, "friction startup"); std::this_thread::sleep_for(5ms);}
  check(std::abs(a - (-.5*9.80665-.3*va+.1)) < 1e-8 &&
    std::abs(b - (-.5*.2*9.80665-.4*vb+.05)) < 1e-8, "friction follows selected joint names despite scrambled interface order");
  va = -.1;
  check(update() == controller_interface::return_type::OK, "reverse friction update");
  end = std::chrono::steady_clock::now() + 150ms;
  while (std::chrono::steady_clock::now() < end) {check(update() == controller_interface::return_type::OK, "effort headroom"); std::this_thread::sleep_for(5ms);}
  check(std::abs(a+4.95) < 1e-8, "optional friction is capped to headroom without dropping gravity or stopping controller");
  va = 0;
  check(update() == controller_interface::return_type::OK && std::abs(a+.5*9.80665) < 1e-8, "friction vanishes immediately when stopped");
  controller.on_deactivate(state); controller.release_interfaces();
  rclcpp::shutdown();
  std::cout << "Gravity controller mapping, zero startup, ramp, damping and fault checks passed\n";
}
