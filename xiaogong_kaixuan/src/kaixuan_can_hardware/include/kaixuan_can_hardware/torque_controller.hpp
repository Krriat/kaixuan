#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace kaixuan_can_hardware
{
class TorqueController : public controller_interface::ControllerInterface
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
  struct Command
  {
    std::array<double, 14> effort{};
    std::int64_t received_ns{0};
    bool valid{false};
  };
  static std::int64_t now_ns();
  void invalidate_interfaces();
  std::vector<std::string> joints_;
  std::vector<std::size_t> interface_order_;
  std::int64_t timeout_ns_{200000000};
  std::int64_t activation_ns_{0};
  std::atomic<bool> accepting_{false};
  realtime_tools::RealtimeBuffer<Command> command_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};
}  // namespace kaixuan_can_hardware
