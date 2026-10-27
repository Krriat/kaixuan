#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

namespace kaixuan_can_hardware
{

class KaixuanHomeServer final : public rclcpp::Node
{
public:
  using FollowJTA = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJTA>;

  KaixuanHomeServer()
  : Node("kaixuan_home_server")
  {
    this->declare_parameter<std::string>("action_name", "/right_arm_controller/follow_joint_trajectory");
    this->declare_parameter<std::vector<std::string>>(
      "joints",
      {"right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6", "right_joint7"});
    this->declare_parameter<std::vector<double>>("home_positions", {0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter<double>("move_time", 5.0);

    const auto action_name = this->get_parameter("action_name").as_string();
    action_client_ = rclcpp_action::create_client<FollowJTA>(this, action_name);

    home_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "home",
      std::bind(&KaixuanHomeServer::on_home, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Home server ready: call /%s/home (Trigger)", this->get_name());
    RCLCPP_INFO(get_logger(), "Using action: %s", action_name.c_str());
  }

private:
  void on_home(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    const auto action_name = this->get_parameter("action_name").as_string();

    if (!action_client_->wait_for_action_server(2s))
    {
      resp->success = false;
      resp->message = "FollowJointTrajectory action server not available: " + action_name;
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    const auto joints = this->get_parameter("joints").as_string_array();
    const auto home = this->get_parameter("home_positions").as_double_array();
    const double move_time = this->get_parameter("move_time").as_double();

    if (home.size() != joints.size())
    {
      resp->success = false;
      resp->message = "home_positions size must match joints size";
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joints;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(home.begin(), home.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(std::max(0.1, move_time));
    traj.points.push_back(pt);

    FollowJTA::Goal goal;
    goal.trajectory = traj;

    rclcpp_action::Client<FollowJTA>::SendGoalOptions options;

    auto goal_future = action_client_->async_send_goal(goal, options);

    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_future, 5s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS)
    {
      resp->success = false;
      resp->message = "Failed to send goal";
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      resp->success = false;
      resp->message = "Goal was rejected";
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    ret = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      result_future,
      std::chrono::duration<double>(move_time + 10.0));

    if (ret != rclcpp::FutureReturnCode::SUCCESS)
    {
      resp->success = false;
      resp->message = "Timed out waiting for trajectory result";
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      resp->success = false;
      resp->message = "Trajectory execution failed";
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    resp->success = true;
    resp->message = "Homing trajectory sent and completed";
    RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
  }

  rclcpp_action::Client<FollowJTA>::SharedPtr action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
};

}  // namespace kaixuan_can_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kaixuan_can_hardware::KaixuanHomeServer>());
  rclcpp::shutdown();
  return 0;
}
