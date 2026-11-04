#include "kaixuan_can_hardware/torque_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include "pluginlib/class_list_macros.hpp"

namespace kaixuan_can_hardware
{
std::int64_t TorqueController::now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

controller_interface::CallbackReturn TorqueController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<double>("command_timeout", 0.2);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TorqueController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {result.names.push_back(joint + "/effort");}
  return result;
}

controller_interface::InterfaceConfiguration TorqueController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn TorqueController::on_configure(const rclcpp_lifecycle::State &)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  const auto timeout = get_node()->get_parameter("command_timeout").as_double();
  const std::set<std::string> unique(joints_.begin(), joints_.end());
  if (joints_.empty() || joints_.size() > 14 || unique.size() != joints_.size() ||
    unique.count("") || !std::isfinite(timeout) || timeout < 0.02 || timeout > 1.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Use 1-14 distinct joints and command_timeout in [0.02, 1.0] seconds");
    return controller_interface::CallbackReturn::ERROR;
  }
  timeout_ns_ = static_cast<std::int64_t>(timeout * 1e9);
  subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::QoS(1).reliable().durability_volatile(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
      if (!accepting_.load()) {return;}
      Command command;
      command.received_ns = now_ns();
      command.valid = message->data.size() == joints_.size() &&
        std::all_of(message->data.begin(), message->data.end(), [](double v) {return std::isfinite(v);});
      if (command.valid) {std::copy(message->data.begin(), message->data.end(), command.effort.begin());}
      // Malformed messages invalidate the previous target immediately.
      command_.writeFromNonRT(command);
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

void TorqueController::invalidate_interfaces()
{
  for (auto & interface : command_interfaces_) {interface.set_value(std::numeric_limits<double>::quiet_NaN());}
}

controller_interface::CallbackReturn TorqueController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != joints_.size()) {return controller_interface::CallbackReturn::ERROR;}
  interface_order_.clear();
  for (const auto & joint : joints_) {
    const auto found = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
      [&joint](const auto & interface) {return interface.get_name() == joint + "/effort";});
    if (found == command_interfaces_.end()) {return controller_interface::CallbackReturn::ERROR;}
    interface_order_.push_back(static_cast<std::size_t>(found - command_interfaces_.begin()));
  }
  command_.writeFromNonRT(Command{});
  invalidate_interfaces();
  activation_ns_ = now_ns();
  accepting_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueController::on_deactivate(const rclcpp_lifecycle::State &)
{
  accepting_.store(false);
  invalidate_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TorqueController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto & command = *command_.readFromRT();
  const auto age = now_ns() - command.received_ns;
  if (!command.valid || command.received_ns < activation_ns_ || age < 0 || age > timeout_ns_) {
    invalidate_interfaces();
  } else {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      command_interfaces_[interface_order_[i]].set_value(command.effort[i]);
    }
  }
  return controller_interface::return_type::OK;
}
}  // namespace kaixuan_can_hardware

PLUGINLIB_EXPORT_CLASS(kaixuan_can_hardware::TorqueController, controller_interface::ControllerInterface)
