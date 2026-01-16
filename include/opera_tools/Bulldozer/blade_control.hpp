// d37pxi_24_blade_control.hpp

#ifndef D37PXI24_BLADE_CONTROL_HPP
#define D37PXI24_BLADE_CONTROL_HPP

#include <memory>
#include <thread>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "com3_msgs/msg/joint_cmd.hpp"

// include は action 生成先に合わせてください
#include "tms_msg_rp/action/tms_rp_bulldozer_blade.hpp"

class D37PXI24BladeControl : public rclcpp::Node
{
public:
  using BladeAction = tms_msg_rp::action::TmsRpBulldozerBlade;
  using GoalHandle  = rclcpp_action::ServerGoalHandle<BladeAction>;

  D37PXI24BladeControl();

private:
  rclcpp_action::Server<BladeAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const BladeAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // pub/sub
  rclcpp::Publisher<com3_msgs::msg::JointCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // latest joint positions (name -> position)
  std::mutex mtx_;
  std::unordered_map<std::string, double> latest_pos_;

  // params
  std::string command_topic_;
  std::string joint_states_topic_;
  double tolerance_{0.02};
  double timeout_sec_{15.0};
  double loop_hz_{20.0};
};

#endif
