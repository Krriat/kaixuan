#include "kaixuan_can_hardware/kaixuan_can_system.hpp"
#include <cstdlib>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
using namespace kaixuan_can_hardware;
void check(bool condition, const char *message)
{if (!condition) {std::cerr << message << '\n'; std::exit(1);}}
hardware_interface::HardwareInfo info(bool cst)
{
  hardware_interface::HardwareInfo h;
  h.name = "test_system"; h.type = "system";
  h.hardware_parameters = {{"control_mode", cst ? "cst" : "csp"}, {"active_arm", "left"}, {"cst_joints", "left_joint7"}};
  for (int i = 0; i < 14; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = std::string(i < 7 ? "right_joint" : "left_joint") + std::to_string(i % 7 + 1);
    if (!cst || i == 13) {
      hardware_interface::InterfaceInfo command; command.name = cst ? "effort" : "position";
      joint.command_interfaces.push_back(command);
    }
    for (const char *name : {"position", "velocity", "effort"}) {
      hardware_interface::InterfaceInfo state; state.name = name; joint.state_interfaces.push_back(state);
    }
    h.joints.push_back(joint);
    h.hardware_parameters[std::string(i < 7 ? "right_gear_ratio" : "left_gear_ratio") + std::to_string(i % 7 + 1)] = "101";
  }
  return h;
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  {
    KaixuanCanSystem hardware;
    check(hardware.on_init(info(true)) == hardware_interface::CallbackReturn::SUCCESS, "CST init without hardware access");
    auto commands = hardware.export_command_interfaces();
    check(commands.size() == 1 && commands[0].get_name() == "left_joint7/effort", "only selected effort interface exported");
    check(hardware.export_state_interfaces().size() == 42, "position velocity effort state interfaces");
    check(hardware.prepare_command_mode_switch({"left_joint6/effort"}, {}) == hardware_interface::return_type::ERROR, "reject unselected joint");
    check(hardware.prepare_command_mode_switch({"left_joint7/position"}, {}) == hardware_interface::return_type::ERROR, "reject position in CST");
    check(hardware.prepare_command_mode_switch({"another_system/effort"}, {}) == hardware_interface::return_type::OK, "ignore foreign interfaces");
    check(hardware.prepare_command_mode_switch({"left_joint7/effort"}, {}) == hardware_interface::return_type::OK, "accept selected effort");
  }
  {
    KaixuanCanSystem hardware;
    check(hardware.on_init(info(false)) == hardware_interface::CallbackReturn::SUCCESS, "CSP init");
    auto commands = hardware.export_command_interfaces();
    check(commands.size() == 14 && commands[13].get_name() == "left_joint7/position", "CSP command compatibility");
  }
  {
    KaixuanCanSystem hardware; auto h = info(true);
    h.hardware_parameters["cst_joints"] = "left_joint7,left_joint7";
    check(hardware.on_init(h) == hardware_interface::CallbackReturn::ERROR, "duplicate CST selection rejected");
  }
  {
    KaixuanCanSystem hardware; auto h = info(true);
    h.joints[13].parameters["effort_limit_nm"] = "nan";
    check(hardware.on_init(h) == hardware_interface::CallbackReturn::ERROR, "invalid limit rejected");
  }
  rclcpp::shutdown();
  std::cout << "CST selection and CSP interface compatibility checks passed\n";
}
