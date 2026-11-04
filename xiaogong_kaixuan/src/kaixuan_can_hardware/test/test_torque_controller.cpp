#include "kaixuan_can_hardware/torque_controller.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
void check(bool condition, const char *message)
{if (!condition) {std::cerr << message << '\n'; std::exit(1);}}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  kaixuan_can_hardware::TorqueController controller;
  check(controller.init("test_torque_controller") == controller_interface::return_type::OK, "controller init");
  controller.get_node()->set_parameter(rclcpp::Parameter("joints", std::vector<std::string>{"joint_a", "joint_b"}));
  controller.get_node()->set_parameter(rclcpp::Parameter("command_timeout", 0.05));
  const rclcpp_lifecycle::State state;
  check(controller.on_configure(state) == controller_interface::CallbackReturn::SUCCESS, "configure");
  double a = 123, b = 456;
  hardware_interface::CommandInterface ia("joint_a", "effort", &a), ib("joint_b", "effort", &b);
  std::vector<hardware_interface::LoanedCommandInterface> interfaces;
  interfaces.emplace_back(ib); interfaces.emplace_back(ia);
  controller.assign_interfaces(std::move(interfaces), {});
  check(controller.on_activate(state) == controller_interface::CallbackReturn::SUCCESS, "activate");
  check(std::isnan(a) && std::isnan(b), "activation invalidates old commands");
  auto publisher_node = std::make_shared<rclcpp::Node>("torque_test_publisher");
  auto publisher = publisher_node->create_publisher<std_msgs::msg::Float64MultiArray>("/test_torque_controller/commands", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.get_node()->get_node_base_interface());
  executor.add_node(publisher_node);
  auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!publisher->get_subscription_count() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(); std::this_thread::sleep_for(5ms);
  }
  check(publisher->get_subscription_count() > 0, "DDS subscription discovered");
  auto send = [&](std::vector<double> data) {
    std_msgs::msg::Float64MultiArray message; message.data = std::move(data);
    publisher->publish(message);
    for (int i = 0; i < 10; ++i) {executor.spin_some(); std::this_thread::sleep_for(1ms);}
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  };
  send({0.1, -0.2});
  check(a == 0.1 && b == -0.2, "command order follows joint names, not loan order");
  std::this_thread::sleep_for(70ms);
  controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  check(std::isnan(a) && std::isnan(b), "publisher timeout invalidates both joints");
  send({0.0, 0.0}); check(a == 0 && b == 0, "fresh command after timeout");
  send({1.0}); check(std::isnan(a) && std::isnan(b), "wrong length invalidates previous target");
  send({0.0, std::numeric_limits<double>::infinity()});
  check(std::isnan(a) && std::isnan(b), "infinite target invalidates all joints");
  send({0.1, 0.2});
  controller.on_deactivate(state);
  check(std::isnan(a) && std::isnan(b), "deactivation invalidates command");
  controller.on_activate(state);
  controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  check(std::isnan(a) && std::isnan(b), "old command not replayed on restart");
  controller.on_deactivate(state); controller.release_interfaces();
  rclcpp::shutdown();
  std::cout << "CST controller mapping, receipt watchdog, malformed input and lifecycle checks passed\n";
}
