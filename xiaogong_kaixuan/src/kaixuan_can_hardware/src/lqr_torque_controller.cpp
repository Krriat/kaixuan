#include "kaixuan_can_hardware/lqr_torque_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <set>
#include "pluginlib/class_list_macros.hpp"

namespace kaixuan_can_hardware
{
double LqrTorqueController::now_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
controller_interface::CallbackReturn LqrTorqueController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  for (auto key : {"effort_limits", "effort_slew", "effort_release_slew", "integral_limits"}) {
    auto_declare<std::vector<double>>(key, {});
  }
  auto_declare<double>("command_timeout", .1);
  auto_declare<double>("runtime_hz", 1000.);
  auto_declare<double>("max_speed_rad_s", .7853981633974483);
  auto_declare<double>("max_tracking_error_rad", .13962634015954636);
  auto_declare<std::int64_t>("input_delay_steps", 3);
  auto_declare<std::int64_t>("input_joint_index", 3);
  auto_declare<double>("input_time_constant_s", .0034);
  auto_declare<std::vector<std::int64_t>>("input_joint_indices", {});
  auto_declare<std::vector<std::int64_t>>("input_delays", {});
  auto_declare<std::vector<double>>("input_time_constants", {});
  auto_declare<std::string>("planning_config_sha256", "");
  auto_declare<bool>("shadow_only", false);
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration LqrTorqueController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {result.names.push_back(joint+"/effort");}
  return result;
}
controller_interface::InterfaceConfiguration LqrTorqueController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    for (auto field : {"position", "velocity", "effort"}) {result.names.push_back(joint+"/"+field);}
  }
  return result;
}
controller_interface::CallbackReturn LqrTorqueController::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    auto node = get_node();
    joints_ = node->get_parameter("joints").as_string_array();
    if (joints_.size() != 7) {throw std::invalid_argument("Exactly seven left joints are required");}
    for (std::size_t i = 0; i < 7; ++i) {
      if (joints_[i] != "left_joint"+std::to_string(i+1)) {throw std::invalid_argument("Left joint order mismatch");}
    }
    auto read = [&](const char * key, LqrServo::Vec & target) {
      auto values = node->get_parameter(key).as_double_array();
      if (values.size() != 7 || !std::all_of(values.begin(), values.end(),
        [](double x) {return std::isfinite(x) && x > 0;})) {throw std::invalid_argument(key);}
      std::copy(values.begin(), values.end(), target.begin());
    };
    read("effort_limits", servo_.limit); read("effort_slew", servo_.increase);
    read("effort_release_slew", servo_.release); read("integral_limits", servo_.integral_limit);
    for (std::size_t i = 0; i < 7; ++i) {
      if (servo_.release[i] < servo_.increase[i] || servo_.integral_limit[i] > servo_.limit[i]) {
        throw std::invalid_argument("Invalid release slew or integral limit");
      }
    }
    const auto delay = node->get_parameter("input_delay_steps").as_int();
    const auto joint = node->get_parameter("input_joint_index").as_int();
    servo_.time_constant = node->get_parameter("input_time_constant_s").as_double();
    timeout_ = node->get_parameter("command_timeout").as_double();
    shadow_only_ = node->get_parameter("shadow_only").as_bool();
    const double hz = node->get_parameter("runtime_hz").as_double();
    max_speed_ = node->get_parameter("max_speed_rad_s").as_double();
    max_error_ = node->get_parameter("max_tracking_error_rad").as_double();
    if (delay < 0 || delay > 10 || joint < 0 || joint >= 7 ||
      !std::isfinite(servo_.time_constant) || servo_.time_constant <= 0 || servo_.time_constant > .05 ||
      !std::isfinite(timeout_) || timeout_ < .02 || timeout_ > .1 || hz != 1000. ||
      !std::isfinite(max_speed_) || max_speed_ <= 0 || !std::isfinite(max_error_) || max_error_ <= 0) {
      throw std::invalid_argument("Invalid servo timing/guards/input model");
    }
    servo_.delay = static_cast<unsigned>(delay); servo_.input_joint = static_cast<unsigned>(joint);
    const auto input_joints = node->get_parameter("input_joint_indices").as_integer_array();
    const auto input_delays = node->get_parameter("input_delays").as_integer_array();
    const auto input_times = node->get_parameter("input_time_constants").as_double_array();
    servo_.channel_count = static_cast<unsigned>(input_joints.size());
    if (input_joints.size() != input_delays.size() || input_joints.size() != input_times.size() ||
      input_joints.size() > 7 || std::set<std::int64_t>(input_joints.begin(), input_joints.end()).size() != input_joints.size()) {
      throw std::invalid_argument("Invalid actuator channel arrays");
    }
    for (unsigned k = 0; k < servo_.channel_count; ++k) {
      if (input_joints[k] < 0 || input_joints[k] >= 7 || input_delays[k] < 0 || input_delays[k] > 10 ||
        !std::isfinite(input_times[k]) || input_times[k] <= 0 || input_times[k] > .05) {
        throw std::invalid_argument("Invalid actuator channel");
      }
      servo_.channels[k] = {static_cast<unsigned>(input_joints[k]), static_cast<unsigned>(input_delays[k]), input_times[k]};
    }
    if (servo_.states() > LqrSetpoint::MAX_STATE) {throw std::invalid_argument("Too many actuator states");}
    nominal_period_ = 1./hz;
    subscription_ = node->create_subscription<Message>("~/setpoints", rclcpp::QoS(1).reliable(),
      [this](Message::ConstSharedPtr message) {
        if (!accepting_.load()) {return;}
        LqrSetpoint s;
        s.received_s = now_s();
        const auto & a = message->data;
        const std::size_t count = 6 + 28 + 7*servo_.states() + 49;
        if (a.size() == count && std::all_of(a.begin(), a.end(), [](double x) {return std::isfinite(x);}) &&
          a[0] == 1 && (a[1] == 1 || a[1] == 2) && a[2] >= 0 && a[2] <= 99 &&
          a[2] == std::floor(a[2]) && a[3] > 0 && a[4] >= 0 && a[4] <= 1 && a[5] >= 0) {
          s.mode = static_cast<int>(a[1]); s.phase = static_cast<int>(a[2]);
          s.sent_s = a[3]; s.scale = a[4]; s.phase_time_s = a[5];
          std::size_t offset = 6;
          for (auto * v : {&s.q, &s.v, &s.feedforward, &s.friction}) {
            std::copy_n(a.begin()+offset, 7, v->begin()); offset += 7;
          }
          for (std::size_t i = 0; i < 7; ++i) {
            std::copy_n(a.begin()+offset, servo_.states(), s.gain.begin()+i*LqrSetpoint::MAX_STATE);
            offset += servo_.states();
          }
          std::copy_n(a.begin()+offset, 49, s.inverse_integral.begin());
          s.valid = true;
        }
        setpoint_.writeFromNonRT(s);  // Invalid frames revoke the previous command.
      });
    publisher_ = std::make_unique<realtime_tools::RealtimePublisher<Message>>(
      node->create_publisher<Message>("~/servo_trace", rclcpp::QoS(1).best_effort()));
    publisher_->msg_.data.resize(5+13*7+2);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "LQR servo configuration: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}
void LqrTorqueController::invalidate()
{
  for (auto & interface : command_interfaces_) {interface.set_value(std::numeric_limits<double>::quiet_NaN());}
}
controller_interface::CallbackReturn LqrTorqueController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 7 || state_interfaces_.size() != 21) {
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < 7; ++i) {
    auto command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
      [&](const auto & x) {return x.get_name() == joints_[i]+"/effort";});
    if (command == command_interfaces_.end()) {return controller_interface::CallbackReturn::ERROR;}
    commands_[i] = static_cast<std::size_t>(command-command_interfaces_.begin());
    for (auto pair : {std::make_pair("position", &positions_), std::make_pair("velocity", &velocities_),
        std::make_pair("effort", &efforts_)}) {
      auto state = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
        [&](const auto & x) {return x.get_name() == joints_[i]+"/"+pair.first;});
      if (state == state_interfaces_.end()) {return controller_interface::CallbackReturn::ERROR;}
      (*pair.second)[i] = static_cast<std::size_t>(state-state_interfaces_.begin());
    }
  }
  setpoint_.writeFromNonRT(LqrSetpoint{}); servo_.reset(); invalidate();
  tripped_ = received_zero_ = false;
  activation_s_ = now_s(); last_update_s_ = last_trace_s_ = 0;
  max_period_s_ = max_compute_s_ = 0;
  accepting_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn LqrTorqueController::on_deactivate(const rclcpp_lifecycle::State &)
{
  accepting_.store(false); invalidate();
  return controller_interface::CallbackReturn::SUCCESS;
}
void LqrTorqueController::trace(const LqrSetpoint & s, double now, double dt,
  const LqrServo::Vec & q, const LqrServo::Vec & v, const LqrServo::Vec & effort)
{
  if (now-last_trace_s_ < .002 || !publisher_->trylock()) {return;}
  auto & a = publisher_->msg_.data;
  a[0] = now; a[1] = s.phase; a[2] = dt; a[3] = now-s.sent_s;
  a[4] = s.phase_time_s + a[3];
  std::size_t offset = 5;
  const std::array<const LqrServo::Vec *, 13> groups{{&q, &v, &s.q, &s.v, &servo_.previous,
    &servo_.raw, &servo_.integral, &effort, &s.feedforward, &servo_.state_feedback,
    &servo_.memory_feedback, &s.friction, &effort}};
  for (const auto * values : groups) {
    std::copy(values->begin(), values->end(), a.begin()+offset); offset += 7;
  }
  a[96] = max_period_s_; a[97] = max_compute_s_;
  last_trace_s_ = now;
  publisher_->unlockAndPublish();
}
controller_interface::return_type LqrTorqueController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const double now = now_s();
  const double dt = last_update_s_ ? now-last_update_s_ : nominal_period_;
  last_update_s_ = now;
  const auto & s = *setpoint_.readFromRT();
  const bool fresh = s.valid && s.received_s >= activation_s_ && s.sent_s >= activation_s_ &&
    now >= s.sent_s && now-s.sent_s <= timeout_ && now-s.received_s <= timeout_;
  if (tripped_ || !fresh || (s.mode == 2 && !received_zero_)) {
    if (received_zero_ || (fresh && s.mode == 2)) {tripped_ = true;}
    invalidate(); return controller_interface::return_type::OK;
  }
  LqrServo::Vec q{}, v{}, effort{};
  for (std::size_t i = 0; i < 7; ++i) {
    q[i] = state_interfaces_[positions_[i]].get_value();
    v[i] = state_interfaces_[velocities_[i]].get_value();
    effort[i] = state_interfaces_[efforts_[i]].get_value();
    if (!std::isfinite(q[i]) || !std::isfinite(v[i]) || std::abs(v[i]) > max_speed_ ||
      std::abs(q[i]-s.q[i]) > max_error_) {
      tripped_ = true; invalidate(); return controller_interface::return_type::OK;
    }
  }
  if (!servo_.step(s, q, v, dt)) {
    tripped_ = true; invalidate(); return controller_interface::return_type::OK;
  }
  if (s.mode == 1) {received_zero_ = true;}
  // Shadow commissioning runs the complete calculation but NEVER sends a
  // finite command, including the zero handshake, to the hardware.
  if (shadow_only_) {invalidate();}
  else {
    for (std::size_t i = 0; i < 7; ++i) {command_interfaces_[commands_[i]].set_value(servo_.previous[i]);}
  }
  max_period_s_ = std::max(max_period_s_, dt);
  max_compute_s_ = std::max(max_compute_s_, now_s()-now);
  trace(s, now, dt, q, v, effort);
  return controller_interface::return_type::OK;
}
}  // namespace kaixuan_can_hardware

PLUGINLIB_EXPORT_CLASS(kaixuan_can_hardware::LqrTorqueController, controller_interface::ControllerInterface)
