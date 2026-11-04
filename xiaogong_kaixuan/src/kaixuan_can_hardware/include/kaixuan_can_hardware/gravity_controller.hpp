#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "kaixuan_can_hardware/gravity_model.hpp"
#include "kaixuan_can_hardware/friction_compensation.hpp"

namespace kaixuan_can_hardware
{
class GravityController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  using Publisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;
  static std::uint64_t now_ns();
  void invalidate();
  GravityModel model_;
  std::vector<std::string> joints_, state_joints_;
  std::vector<std::size_t> model_order_, command_order_, position_order_, velocity_order_;
  std::vector<double> positions_, velocities_, damping_, limits_, commands_;
  std::vector<FrictionCompensation> friction_;
  std::vector<double> friction_commands_;
  std::unique_ptr<Publisher> gravity_publisher_, command_publisher_, friction_publisher_;
  double scale_{1.0}, zero_hold_{0.5}, ramp_time_{5.0};
  bool friction_enabled_{false};
  double friction_scale_{0}, friction_deadband_{0}, friction_full_speed_{0}, friction_filter_time_{0};
  std::uint64_t activation_ns_{0}, previous_ns_{0}, published_ns_{0};
};
}  // namespace kaixuan_can_hardware
