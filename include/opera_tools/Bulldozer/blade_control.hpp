#ifndef BULLDOZER_BLADE_CONTROL_HPP
#define BULLDOZER_BLADE_CONTROL_HPP

#include <memory>
#include <thread>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "com3_msgs/msg/joint_cmd.hpp"

#include "tms_msg_rp/action/tms_rp_bulldozer_blade.hpp"

class BulldozerBladeControl : public rclcpp::Node
{
public:
  using BladeAction = tms_msg_rp::action::TmsRpBulldozerBlade;
  using GoalHandle  = rclcpp_action::ServerGoalHandle<BladeAction>;

  BulldozerBladeControl();

private:
  // ============================================================
  // Action server
  // ============================================================
  rclcpp_action::Server<BladeAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const BladeAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // ============================================================
  // Publisher / Subscriber
  // ============================================================
  rclcpp::Publisher<com3_msgs::msg::JointCmd>::SharedPtr cmd_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;

  void joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  // ============================================================
  // Latest joint positions
  // ============================================================
  std::mutex mtx_;

  std::unordered_map<std::string, double> latest_pos_;

  // ============================================================
  // Blade hold
  //
  // blade_hold=true のActionが完了した後、
  // ここに保持用のJointCmdを保存して送信し続ける。
  // ============================================================
  std::mutex hold_mtx_;

  bool blade_hold_active_{false};

  com3_msgs::msg::JointCmd hold_cmd_;

  // 保持用JointCmdを定期送信するTimer
  rclcpp::TimerBase::SharedPtr hold_timer_;

  void publish_hold_command();

  // ============================================================
  // Parameters
  // ============================================================
  std::string command_topic_;
  std::string joint_states_topic_;
  std::string control_type_;

  double velocity_kp_;
  double effort_kp_;
  double max_velocity_;
  double max_effort_;

  double tolerance_{0.02};
  double timeout_sec_{15.0};
  double loop_hz_{20.0};
};

#endif