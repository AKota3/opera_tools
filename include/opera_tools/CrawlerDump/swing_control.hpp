#ifndef CRAWLERDUMP_SWING_CONTROL_HPP
#define CRAWLERDUMP_SWING_CONTROL_HPP

#include <memory>
#include <thread>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "com3_msgs/msg/joint_cmd.hpp"

#include "tms_msg_rp/action/tms_rp_crawler_dump_swing_angle.hpp"

class SwingControl : public rclcpp::Node
{
public:
  using SwingAction = tms_msg_rp::action::TmsRpCrawlerDumpSwingAngle;
  using GoalHandle  = rclcpp_action::ServerGoalHandle<SwingAction>;

  SwingControl();

private:
  // action server
  rclcpp_action::Server<SwingAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const SwingAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // pub/sub
  rclcpp::Publisher<com3_msgs::msg::JointCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // latest joint position
  std::mutex mtx_;
  std::optional<double> latest_rotate_joint_pos_;  // rad

  // parameters
  std::string command_topic_;
  std::string joint_states_topic_;
  std::string controlled_joint_;
  double tolerance_rad_;
  double timeout_sec_;
  double loop_hz_;
};

#endif
