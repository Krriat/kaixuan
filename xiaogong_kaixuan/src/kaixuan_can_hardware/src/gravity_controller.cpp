#include "kaixuan_can_hardware/gravity_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"

namespace kaixuan_can_hardware
{
std::uint64_t GravityController::now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
controller_interface::CallbackReturn GravityController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::vector<double>>("gravity_vector", {0.0, 0.0, -9.80665});
  auto_declare<std::vector<double>>("damping", {});
  auto_declare<std::vector<double>>("effort_limits", {});
  auto_declare<double>("gravity_scale", 1.0);
  auto_declare<double>("zero_hold_duration", 0.5);
  auto_declare<double>("ramp_duration", 5.0);
  auto_declare<std::string>("payload_frame", "left_ee");
  auto_declare<double>("payload_mass", 0.0);
  auto_declare<std::vector<double>>("payload_com", {0.0, 0.0, 0.0});
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration GravityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joints_) {result.names.push_back(name + "/effort");}
  return result;
}
controller_interface::InterfaceConfiguration GravityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : state_joints_) {
    result.names.push_back(name + "/position");
    result.names.push_back(name + "/velocity");
  }
  return result;
}
controller_interface::CallbackReturn GravityController::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    auto node = get_node();
    joints_ = node->get_parameter("joints").as_string_array();
    damping_ = node->get_parameter("damping").as_double_array();
    limits_ = node->get_parameter("effort_limits").as_double_array();
    scale_ = node->get_parameter("gravity_scale").as_double();
    zero_hold_ = node->get_parameter("zero_hold_duration").as_double();
    ramp_time_ = node->get_parameter("ramp_duration").as_double();
    const std::set<std::string> unique(joints_.begin(), joints_.end());
    if (joints_.empty() || joints_.size() > 14 || unique.size() != joints_.size() ||
      !std::isfinite(scale_) || scale_ <= 0 || scale_ > 1 ||
      !std::isfinite(zero_hold_) || zero_hold_ < 0.2 || zero_hold_ > 5 ||
      !std::isfinite(ramp_time_) || ramp_time_ < 1 || ramp_time_ > 60 ||
      damping_.size() != joints_.size() || limits_.size() != joints_.size())
    {throw std::invalid_argument("Invalid joints, damping, limits, gravity scale or ramp timing");}
    const auto gravity = node->get_parameter("gravity_vector").as_double_array();
    const auto com = node->get_parameter("payload_com").as_double_array();
    if (gravity.size() != 3 || com.size() != 3) {throw std::invalid_argument("Use 3D gravity and payload vectors");}
    model_.configure(node->get_parameter("robot_description").as_string(),
      {gravity[0], gravity[1], gravity[2]}, node->get_parameter("payload_frame").as_string(),
      node->get_parameter("payload_mass").as_double(), {com[0], com[1], com[2]});
    state_joints_ = model_.joints();
    model_order_.clear();
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      auto found = std::find(state_joints_.begin(), state_joints_.end(), joints_[i]);
      if (found == state_joints_.end() || !std::isfinite(damping_[i]) || damping_[i] < 0 ||
        !std::isfinite(limits_[i]) || limits_[i] <= 0)
      {throw std::invalid_argument("Unknown joint or invalid effort/damping parameter");}
      model_order_.push_back(static_cast<std::size_t>(found - state_joints_.begin()));
    }
    positions_.resize(state_joints_.size()); velocities_.resize(state_joints_.size());
    commands_.resize(joints_.size());
    gravity_publisher_ = std::make_unique<Publisher>(node->create_publisher<std_msgs::msg::Float64MultiArray>("~/gravity_torques", 1));
    command_publisher_ = std::make_unique<Publisher>(node->create_publisher<std_msgs::msg::Float64MultiArray>("~/commanded_torques", 1));
    gravity_publisher_->msg_.data.resize(joints_.size());
    command_publisher_->msg_.data.resize(joints_.size());
    RCLCPP_INFO(node->get_logger(), "Pinocchio gravity compensation: %zu joints, scale=%.3f, zero=%.2fs, ramp=%.2fs",
      joints_.size(), scale_, zero_hold_, ramp_time_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Gravity controller configuration failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}
void GravityController::invalidate()
{
  for (auto & interface : command_interfaces_) {interface.set_value(std::numeric_limits<double>::quiet_NaN());}
}
controller_interface::CallbackReturn GravityController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != joints_.size() || state_interfaces_.size() != 2 * state_joints_.size()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  command_order_.clear(); position_order_.clear(); velocity_order_.clear();
  const auto find = [](const auto & interfaces, const std::string & name) {
      auto it = std::find_if(interfaces.begin(), interfaces.end(),
        [&name](const auto & interface) {return interface.get_name() == name;});
      if (it == interfaces.end()) {throw std::invalid_argument("Missing interface: " + name);}
      return static_cast<std::size_t>(it - interfaces.begin());
    };
  try {
    for (const auto & name : joints_) {command_order_.push_back(find(command_interfaces_, name + "/effort"));}
    for (const auto & name : state_joints_) {
      position_order_.push_back(find(state_interfaces_, name + "/position"));
      velocity_order_.push_back(find(state_interfaces_, name + "/velocity"));
    }
    invalidate();
    activation_ns_ = previous_ns_ = now_ns(); published_ns_ = 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn GravityController::on_deactivate(const rclcpp_lifecycle::State &)
{
  invalidate();
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::return_type GravityController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    const auto now = now_ns();
    if (now < previous_ns_ || now - previous_ns_ > 50000000ULL) {
      throw std::runtime_error("Controller update gap exceeded 50 ms");
    }
    previous_ns_ = now;
    for (std::size_t i = 0; i < positions_.size(); ++i) {
      positions_[i] = state_interfaces_[position_order_[i]].get_value();
      velocities_[i] = state_interfaces_[velocity_order_[i]].get_value();
      if (!std::isfinite(velocities_[i])) {throw std::runtime_error("Nonfinite joint velocity");}
    }
    const auto & gravity = model_.compute(positions_);
    const double elapsed = (now - activation_ns_) * 1e-9;
    const double ramp = std::clamp((elapsed - zero_hold_) / ramp_time_, 0.0, 1.0);
    // Validate every joint before writing any command. Never silently under-compensate by clipping.
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const double demand = scale_ * gravity[model_order_[i]] - damping_[i] * velocities_[model_order_[i]];
      if (!std::isfinite(demand) || std::abs(demand) > limits_[i]) {
        throw std::runtime_error("Required compensation exceeds configured effort limit");
      }
      commands_[i] = elapsed < zero_hold_ ? 0.0 :
        ramp * scale_ * gravity[model_order_[i]] - damping_[i] * velocities_[model_order_[i]];
      if (std::abs(commands_[i]) > limits_[i]) {
        throw std::runtime_error("Ramped compensation exceeds configured effort limit");
      }
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      command_interfaces_[command_order_[i]].set_value(commands_[i]);
    }
    if (now - published_ns_ >= 50000000ULL) {
      published_ns_ = now;
      if (gravity_publisher_->trylock()) {
        for (std::size_t i = 0; i < joints_.size(); ++i) {gravity_publisher_->msg_.data[i] = gravity[model_order_[i]];}
        gravity_publisher_->unlockAndPublish();
      }
      if (command_publisher_->trylock()) {
        for (std::size_t i = 0; i < joints_.size(); ++i) {command_publisher_->msg_.data[i] = commands_[i];}
        command_publisher_->unlockAndPublish();
      }
    }
  } catch (const std::exception & e) {
    invalidate();
    RCLCPP_ERROR(get_node()->get_logger(), "Gravity compensation stopped: %s", e.what());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}
}  // namespace kaixuan_can_hardware
PLUGINLIB_EXPORT_CLASS(kaixuan_can_hardware::GravityController, controller_interface::ControllerInterface)
