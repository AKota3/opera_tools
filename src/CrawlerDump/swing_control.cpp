#include "opera_tools/CrawlerDump/swing_control.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

SwingControl::SwingControl()
: rclcpp::Node("tms_if_crawlerdump_swing_node")
{
  // ---- parameters ----
  command_topic_      = this->declare_parameter<std::string>("command_topic", "/mst110cr/rot_dump_cmd");
  joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/mst110cr/joint_states");
  controlled_joint_   = this->declare_parameter<std::string>("controlled_joint", "rotate_joint");

  tolerance_rad_ = this->declare_parameter<double>("tolerance_rad", 0.05);
  timeout_sec_   = this->declare_parameter<double>("timeout_sec", 30.0);
  loop_hz_       = this->declare_parameter<double>("loop_hz", 20.0);

  // ---- publisher/subscriber ----
  cmd_pub_ = this->create_publisher<com3_msgs::msg::JointCmd>(
    command_topic_, rclcpp::QoS(10).reliable());

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::QoS(50).best_effort(),
    std::bind(&SwingControl::joint_state_callback, this, std::placeholders::_1));

  // ---- action server ----
  action_server_ = rclcpp_action::create_server<SwingAction>(
    this,
    "set_swing_angle",
    std::bind(&SwingControl::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SwingControl::handle_cancel, this, std::placeholders::_1),
    std::bind(&SwingControl::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "SwingControl node started.");
}

rclcpp_action::GoalResponse SwingControl::handle_goal(
  const rclcpp_action::GoalUUID& /*uuid*/,
  std::shared_ptr<const SwingAction::Goal> goal)
{
  if (!goal) {
    RCLCPP_WARN(this->get_logger(), "Received null goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!std::isfinite(goal->target_angle)) {
    RCLCPP_WARN(this->get_logger(), "Reject goal: target_angle is not finite");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->control_type > 2) {
    RCLCPP_WARN(this->get_logger(), "Reject goal: control_type is invalid (%u)", goal->control_type);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->control_type == 1 && !std::isfinite(goal->velocity)) {
    RCLCPP_WARN(this->get_logger(), "Reject goal: velocity is not finite (control_type=1)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->control_type == 2 && !std::isfinite(goal->effort)) {
    RCLCPP_WARN(this->get_logger(), "Reject goal: effort is not finite (control_type=2)");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SwingControl::handle_cancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SwingControl::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&SwingControl::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void SwingControl::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg) return;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == controlled_joint_) {
      if (i < msg->position.size() && std::isfinite(msg->position[i])) {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_rotate_joint_pos_ = msg->position[i];
      }
      return;
    }
  }
}

void SwingControl::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "swing_control execute start");

  auto result = std::make_shared<SwingAction::Result>();
  const auto goal = goal_handle->get_goal();

  const double target_angle = goal->target_angle;
  const uint8_t control_type = goal->control_type;

  com3_msgs::msg::JointCmd cmd;
  cmd.joint_name   = {controlled_joint_};
  cmd.control_type = control_type;

  cmd.position = {-1.0};
  cmd.velocity = {-1.0};
  cmd.effort   = {-1.0};

  if (control_type == 0) {
    cmd.position = {target_angle};
  } else if (control_type == 1) {
    cmd.velocity = {goal->velocity};
  } else {
    cmd.effort = {goal->effort};
  }

  rclcpp::Rate rate(loop_hz_);
  const auto start = this->now();

  while (rclcpp::ok())
  {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    cmd_pub_->publish(cmd);

    std::optional<double> current_opt;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      current_opt = latest_rotate_joint_pos_;
    }

    if (current_opt.has_value()) {
      const double current = *current_opt;
      const double err = target_angle - current;

      auto fb = std::make_shared<SwingAction::Feedback>();
      fb->current_error = err;
      goal_handle->publish_feedback(fb);

      if (std::fabs(err) <= tolerance_rad_) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),
          "Reached target: target=%.6f current=%.6f err=%.6f tol=%.6f",
          target_angle, current, err, tolerance_rad_);
        return;
      }
    }

    const double elapsed = (this->now() - start).seconds();
    if (elapsed > timeout_sec_) {
      result->success = false;
      goal_handle->abort(result);
      RCLCPP_WARN(this->get_logger(), "Timeout: %.3f sec (target=%.6f)", elapsed, target_angle);
      return;
    }

    rate.sleep();
  }

  if (goal_handle->is_active()) {
    result->success = false;
    goal_handle->abort(result);
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwingControl>());
  rclcpp::shutdown();
  return 0;
}
