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
  auto_declare<bool>("friction_enabled", false);
  auto_declare<double>("friction_scale", 0.25);
  auto_declare<double>("friction_velocity_deadband_rad_s", 0.003490658503988659);
  auto_declare<double>("friction_velocity_full_rad_s", 0.008726646259971648);
  auto_declare<double>("friction_filter_time_constant_s", 0.02);
  auto_declare<std::vector<std::string>>("friction_models", {});
  for (const auto * name : {"friction_coulomb_nm", "friction_viscous_nm_s_per_rad",
      "friction_min_speed_rad_s", "friction_max_speed_rad_s", "friction_effort_limits_nm",
      "friction_slew_nm_s", "friction_lookup_speeds_rad_s", "friction_lookup_torques_nm"})
  {auto_declare<std::vector<double>>(name, {});}
  auto_declare<std::vector<std::int64_t>>("friction_lookup_offsets", {});
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
    friction_.assign(joints_.size(), FrictionCompensation{});
    friction_commands_.assign(joints_.size(), 0.0);
    friction_enabled_ = node->get_parameter("friction_enabled").as_bool();
    friction_scale_ = node->get_parameter("friction_scale").as_double();
    friction_deadband_ = node->get_parameter("friction_velocity_deadband_rad_s").as_double();
    friction_full_speed_ = node->get_parameter("friction_velocity_full_rad_s").as_double();
    friction_filter_time_ = node->get_parameter("friction_filter_time_constant_s").as_double();
    if (!std::isfinite(friction_scale_) || friction_scale_ < 0 || friction_scale_ > 1 ||
      !std::isfinite(friction_deadband_) || friction_deadband_ < 0 ||
      !std::isfinite(friction_full_speed_) || friction_full_speed_ <= friction_deadband_ ||
      !std::isfinite(friction_filter_time_) || friction_filter_time_ < 0 || friction_filter_time_ > 1)
    {throw std::invalid_argument("Invalid friction scale/deadband/filter settings");}
    if (friction_enabled_) {
      const auto models = node->get_parameter("friction_models").as_string_array();
      const auto values = [&](const char * name) {
          auto result = node->get_parameter(name).as_double_array();
          if (result.size() != joints_.size()) {throw std::invalid_argument(std::string("Wrong size: ") + name);}
          return result;
        };
      const auto fc = values("friction_coulomb_nm"), fv = values("friction_viscous_nm_s_per_rad");
      const auto vmin = values("friction_min_speed_rad_s"), vmax = values("friction_max_speed_rad_s");
      const auto caps = values("friction_effort_limits_nm"), slew = values("friction_slew_nm_s");
      const auto speeds = node->get_parameter("friction_lookup_speeds_rad_s").as_double_array();
      const auto torques = node->get_parameter("friction_lookup_torques_nm").as_double_array();
      const auto offsets = node->get_parameter("friction_lookup_offsets").as_integer_array();
      if (models.size() != joints_.size() || offsets.size() != joints_.size()+1 ||
        speeds.size() != torques.size() || offsets.front() != 0 ||
        offsets.back() != static_cast<std::int64_t>(speeds.size()))
      {throw std::invalid_argument("Invalid friction models or lookup offsets");}
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (offsets[i] < 0 || offsets[i+1] < offsets[i] ||
          offsets[i+1] > static_cast<std::int64_t>(speeds.size()))
        {throw std::invalid_argument("Invalid friction lookup span");}
        auto & p = friction_[i].parameters;
        p.model = models[i]; p.coulomb_nm = fc[i]; p.viscous_nm_s_per_rad = fv[i];
        p.min_speed_rad_s = vmin[i]; p.max_speed_rad_s = vmax[i];
        p.limit_nm = caps[i]; p.slew_nm_s = slew[i];
        p.speeds.assign(speeds.begin()+offsets[i], speeds.begin()+offsets[i+1]);
        p.torques.assign(torques.begin()+offsets[i], torques.begin()+offsets[i+1]);
        p.validate(limits_[i]);
      }
    }
    gravity_publisher_ = std::make_unique<Publisher>(node->create_publisher<std_msgs::msg::Float64MultiArray>("~/gravity_torques", 1));
    command_publisher_ = std::make_unique<Publisher>(node->create_publisher<std_msgs::msg::Float64MultiArray>("~/commanded_torques", 1));
    friction_publisher_ = std::make_unique<Publisher>(node->create_publisher<std_msgs::msg::Float64MultiArray>("~/friction_torques", 1));
    gravity_publisher_->msg_.data.resize(joints_.size());
    command_publisher_->msg_.data.resize(joints_.size());
    friction_publisher_->msg_.data.resize(joints_.size());
    RCLCPP_INFO(node->get_logger(), "Pinocchio gravity compensation: %zu joints, scale=%.3f, zero=%.2fs, ramp=%.2fs",
      joints_.size(), scale_, zero_hold_, ramp_time_);
    RCLCPP_INFO(node->get_logger(), "Friction compensation: enabled=%s, scale=%.3f, velocity deadband=%.3f deg/s",
      friction_enabled_ ? "true" : "false", friction_scale_, friction_deadband_*180.0/3.141592653589793);
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
    for (auto & friction : friction_) {friction.reset();}
    std::fill(friction_commands_.begin(), friction_commands_.end(), 0.0);
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
  for (auto & friction : friction_) {friction.reset();}
  std::fill(friction_commands_.begin(), friction_commands_.end(), 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::return_type GravityController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    const auto now = now_ns();
    if (now < previous_ns_ || now - previous_ns_ > 50000000ULL) {
      throw std::runtime_error("Controller update gap exceeded 50 ms");
    }
    const double dt = (now-previous_ns_) * 1e-9;
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
      const double base = ramp * scale_ * gravity[model_order_[i]] - damping_[i] * velocities_[model_order_[i]];
      if (!std::isfinite(base) || std::abs(base) > limits_[i]) {
        throw std::runtime_error("Ramped compensation exceeds configured effort limit");
      }
      if (elapsed < zero_hold_) {
        friction_[i].reset(); friction_commands_[i] = commands_[i] = 0.0;
      } else {
        const double requested = friction_enabled_ ? friction_[i].update(
          velocities_[model_order_[i]], dt, ramp, friction_scale_,
          friction_deadband_, friction_full_speed_, friction_filter_time_) : 0.0;
        // Only trim optional friction assistance to available effort headroom;
        // gravity/damping overflow still stops the entire controller as before.
        const double lower = std::max(-limits_[i]-demand, -limits_[i]-base);
        const double upper = std::min(limits_[i]-demand, limits_[i]-base);
        friction_commands_[i] = std::clamp(requested, lower, upper);
        friction_[i].track_applied(friction_commands_[i]);
        commands_[i] = base + friction_commands_[i];
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
      if (friction_publisher_->trylock()) {
        for (std::size_t i = 0; i < joints_.size(); ++i) {friction_publisher_->msg_.data[i] = friction_commands_[i];}
        friction_publisher_->unlockAndPublish();
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
