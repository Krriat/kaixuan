#include "kaixuan_can_hardware/lqr_torque_controller.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
void check(bool value, const char * message)
{if (!value) {std::cerr << message << '\n'; std::exit(1);}}
double steady_s()
{return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  kaixuan_can_hardware::LqrTorqueController controller;
  check(controller.init("test_lqr_controller") == controller_interface::return_type::OK, "init");
  auto node = controller.get_node();
  std::vector<std::string> joints;
  for (int i = 1; i <= 7; ++i) {joints.push_back("left_joint"+std::to_string(i));}
  node->set_parameter(rclcpp::Parameter("joints", joints));
  node->set_parameter(rclcpp::Parameter("command_timeout", .03));
  for (auto key : {"effort_limits", "integral_limits"}) {
    node->set_parameter(rclcpp::Parameter(key, std::vector<double>(7, 1.)));
  }
  for (auto key : {"effort_slew", "effort_release_slew"}) {
    node->set_parameter(rclcpp::Parameter(key, std::vector<double>(7, 1000.)));
  }
  const rclcpp_lifecycle::State state;
  check(controller.on_configure(state) == controller_interface::CallbackReturn::SUCCESS, "configure");
  std::array<double, 7> output{}, q{}, v{}, effort{};
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  commands.reserve(7); states.reserve(21);
  for (int i = 6; i >= 0; --i) {
    commands.emplace_back(joints[i], "effort", &output[i]);
    states.emplace_back(joints[i], "effort", &effort[i]);
    states.emplace_back(joints[i], "velocity", &v[i]);
    states.emplace_back(joints[i], "position", &q[i]);
  }
  std::vector<hardware_interface::LoanedCommandInterface> lc;
  std::vector<hardware_interface::LoanedStateInterface> ls;
  for (auto & c : commands) {lc.emplace_back(c);}
  for (auto & s : states) {ls.emplace_back(s);}
  controller.assign_interfaces(std::move(lc), std::move(ls));
  auto activate = [&]() {
    check(controller.on_activate(state) == controller_interface::CallbackReturn::SUCCESS, "activate");
    check(std::isnan(output[0]), "activation invalidates commands");
  };
  activate();
  auto pubnode = std::make_shared<rclcpp::Node>("native_lqr_test_publisher");
  auto pub = pubnode->create_publisher<std_msgs::msg::Float64MultiArray>("/test_lqr_controller/setpoints", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface()); executor.add_node(pubnode);
  auto deadline = std::chrono::steady_clock::now()+3s;
  while (!pub->get_subscription_count() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(); std::this_thread::sleep_for(1ms);
  }
  check(pub->get_subscription_count() == 1, "DDS discovery");
  auto tick = [&]() {
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.001));
  };
  auto frame = [&](double mode) {
    std::vector<double> a(6+28+7*25+49, 0.);
    a[0]=1; a[1]=mode; a[2]=1; a[3]=steady_s(); a[4]=1;
    for (unsigned i=0; i<7; ++i) {
      a[34+i*25+i]=100; a[34+i*25+7+i]=20; a[34+i*25+14+i]=5;
      a[34+7*25+i*7+i]=.2;
    }
    return a;
  };
  auto send = [&](std::vector<double> a) {
    std_msgs::msg::Float64MultiArray message; message.data=std::move(a); pub->publish(message);
    for (int i=0; i<7; ++i) {executor.spin_some(); tick(); std::this_thread::sleep_for(1ms);}
  };
  send(frame(2)); check(std::isnan(output[0]), "feedback cannot bypass explicit zero handshake");
  send(frame(1)); check(std::isnan(output[0]), "trip cannot rearm via another topic frame");
  controller.on_deactivate(state); activate();
  send(frame(1)); check(output[0] == 0 && output[6] == 0, "zero handshake");
  send(frame(2));
  q[3]=.001;
  tick(); check(output[3] < -.099 && output[3] > -.101, "feedback uses fresh state without a new DDS setpoint");
  check(std::abs(output[2]) < 1e-10, "joint-name mapping");
  v[3]=1.; tick(); check(std::isnan(output[3]), "native speed guard");
  v[3]=q[3]=0; send(frame(1)); check(std::isnan(output[3]), "guard stop latched");
  controller.on_deactivate(state); activate(); send(frame(1)); send(frame(2));
  std::this_thread::sleep_for(40ms); tick(); check(std::isnan(output[0]), "monotonic timeout stops output");
  controller.on_deactivate(state); activate(); send(frame(1));
  auto stale=frame(2); stale[3]-=1.; send(stale);
  check(std::isnan(output[0]), "fresh DDS receipt does not make an old generated setpoint fresh");
  controller.on_deactivate(state); activate(); send(frame(1)); send({1.,2.});
  check(std::isnan(output[0]), "malformed packet invalidates previous setpoint");
  controller.on_deactivate(state);
  node->set_parameter(rclcpp::Parameter("shadow_only", true));
  check(controller.on_configure(state) == controller_interface::CallbackReturn::SUCCESS, "shadow configure");
  activate(); send(frame(1)); send(frame(2)); q[3]=.001; tick();
  check(std::all_of(output.begin(), output.end(), [](double x) {return std::isnan(x);}),
    "shadow mode must never deliver zero or finite torque to hardware");
  controller.on_deactivate(state); controller.release_interfaces(); rclcpp::shutdown();
  std::cout << "Native LQI lifecycle, current-state feedback, mapping, handshake, guards and watchdog passed\n";
}
