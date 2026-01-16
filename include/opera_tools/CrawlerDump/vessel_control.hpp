#ifndef CRAWLERDUMP_VESSEL_CONTROL_HPP
#define CRAWLERDUMP_VESSEL_CONTROL_HPP

#include <memory>
#include <thread>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "com3_msgs/msg/joint_cmd.hpp"

#include "tms_msg_rp/action/tms_rp_crawler_dump_dump_angle.hpp"

class VesselControl : public rclcpp::Node
{
public:
  using VesselAction = tms_msg_rp::action::TmsRpCrawlerDumpDumpAngle;
  using GoalHandle  = rclcpp_action::ServerGoalHandle<VesselAction>;

  VesselControl();

private:
  // action server
  rclcpp_action::Server<VesselAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const VesselAction::Goal> goal);

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
  std::optional<double> latest_vessel_joint_pos_;  // rad

  // parameters
  std::string command_topic_;
  std::string joint_states_topic_;
  std::string controlled_joint_;
  double tolerance_rad_;
  double timeout_sec_;
  double loop_hz_;
};

#endif
