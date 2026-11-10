#pragma once
#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "kaixuan_can_hardware/lqr_servo.hpp"

namespace kaixuan_can_hardware
{
class LqrTorqueController : public controller_interface::ControllerInterface
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
  using Message = std_msgs::msg::Float64MultiArray;
  static double now_s();
  void invalidate();
  void trace(const LqrSetpoint &, double, double, const LqrServo::Vec &, const LqrServo::Vec &,
    const LqrServo::Vec &);
  std::vector<std::string> joints_;
  std::array<std::size_t, 7> commands_{}, positions_{}, velocities_{}, efforts_{};
  LqrServo servo_;
  double timeout_{.1}, max_speed_{.7853981633974483}, max_error_{.13962634015954636};
  double nominal_period_{.001}, activation_s_{0}, last_update_s_{0}, last_trace_s_{0};
  double max_period_s_{0}, max_compute_s_{0};
  bool shadow_only_{false};
  bool tripped_{false}, received_zero_{false};
  std::atomic<bool> accepting_{false};
  realtime_tools::RealtimeBuffer<LqrSetpoint> setpoint_;
  rclcpp::Subscription<Message>::SharedPtr subscription_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Message>> publisher_;
};
}  // namespace kaixuan_can_hardware
