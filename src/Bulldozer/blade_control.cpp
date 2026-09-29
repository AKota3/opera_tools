#include "opera_tools/Bulldozer/blade_control.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// ============================================================
// Constructor
// ============================================================
BulldozerBladeControl::BulldozerBladeControl()
: rclcpp::Node("bulldozer_blade_control_node")
{
  // ---- parameters ----
  command_topic_ =
    this->declare_parameter<std::string>(
      "command_topic", "blade_cmd");

  joint_states_topic_ =
    this->declare_parameter<std::string>(
      "joint_states_topic", "joint_states");

  tolerance_ =
    this->declare_parameter<double>(
      "tolerance", 0.15);

  timeout_sec_ =
    this->declare_parameter<double>(
      "timeout_sec", 15.0);

  loop_hz_ =
    this->declare_parameter<double>(
      "loop_hz", 20.0);

  // ============================================================
  // Publisher
  // ============================================================
  cmd_pub_ =
    this->create_publisher<com3_msgs::msg::JointCmd>(
      command_topic_,
      rclcpp::QoS(10).reliable());

  // ============================================================
  // Subscriber
  // ============================================================
  joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      rclcpp::QoS(50).best_effort(),
      std::bind(
        &BulldozerBladeControl::joint_state_callback,
        this,
        std::placeholders::_1));

  // ============================================================
  // Hold timer
  //
  // blade_hold=true で目標に到達した後、
  // Action execute() が終了しても、
  // このTimerからJointCmdを送り続ける。
  // ============================================================
  const auto hold_period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / loop_hz_));

  hold_timer_ =
    this->create_wall_timer(
      hold_period,
      std::bind(
        &BulldozerBladeControl::publish_hold_command,
        this));

  // ============================================================
  // Action server
  // ============================================================
  action_server_ =
    rclcpp_action::create_server<BladeAction>(
      this,
      "set_bulldozer_blade",

      std::bind(
        &BulldozerBladeControl::handle_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),

      std::bind(
        &BulldozerBladeControl::handle_cancel,
        this,
        std::placeholders::_1),

      std::bind(
        &BulldozerBladeControl::handle_accepted,
        this,
        std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "BulldozerBladeControl started.");

  RCLCPP_INFO(
    this->get_logger(),
    " command_topic: %s",
    command_topic_.c_str());

  RCLCPP_INFO(
    this->get_logger(),
    " joint_states_topic: %s",
    joint_states_topic_.c_str());
}


// ============================================================
// handle_goal
// ============================================================
rclcpp_action::GoalResponse
BulldozerBladeControl::handle_goal(
  const rclcpp_action::GoalUUID& /*uuid*/,
  std::shared_ptr<const BladeAction::Goal> goal)
{
  if (!goal) {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject: null goal");

    return rclcpp_action::GoalResponse::REJECT;
  }

  // ------------------------------------------------------------
  // control_type
  // ------------------------------------------------------------
  if (goal->control_type > 2) {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject: invalid control_type=%u",
      goal->control_type);

    return rclcpp_action::GoalResponse::REJECT;
  }

  // ------------------------------------------------------------
  // joint_name
  // ------------------------------------------------------------
  const size_t n = goal->joint_name.size();

  if (n == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject: joint_name is empty");

    return rclcpp_action::GoalResponse::REJECT;
  }

  // ------------------------------------------------------------
  // goal_position
  // ------------------------------------------------------------
  if (goal->goal_position.size() != n) {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject: goal_position size mismatch (need %zu)",
      n);

    return rclcpp_action::GoalResponse::REJECT;
  }

  // ------------------------------------------------------------
  // velocity
  // ------------------------------------------------------------
  if (goal->control_type == 1 &&
      goal->velocity.size() != n)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject: velocity size mismatch (need %zu)",
      n);

    return rclcpp_action::GoalResponse::REJECT;
  }

  // ------------------------------------------------------------
  // effort
  // ------------------------------------------------------------
  if (goal->control_type == 2 &&
      goal->effort.size() != n)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject: effort size mismatch (need %zu)",
      n);

    return rclcpp_action::GoalResponse::REJECT;
  }

  // ------------------------------------------------------------
  // finite check
  // ------------------------------------------------------------
  for (size_t i = 0; i < n; ++i)
  {
    if (!std::isfinite(goal->goal_position[i])) {
      RCLCPP_WARN(
        this->get_logger(),
        "Reject: goal_position[%zu] is not finite",
        i);

      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->control_type == 1 &&
        !std::isfinite(goal->velocity[i]))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Reject: velocity[%zu] is not finite",
        i);

      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->control_type == 2 &&
        !std::isfinite(goal->effort[i]))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Reject: effort[%zu] is not finite",
        i);

      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Accept goal: blade_hold=%s",
    goal->blade_hold ? "true" : "false");

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


// ============================================================
// handle_cancel
// ============================================================
rclcpp_action::CancelResponse
BulldozerBladeControl::handle_cancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Cancel requested");

  return rclcpp_action::CancelResponse::ACCEPT;
}


// ============================================================
// handle_accepted
// ============================================================
void BulldozerBladeControl::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{
    std::bind(
      &BulldozerBladeControl::execute,
      this,
      std::placeholders::_1),
    goal_handle
  }.detach();
}


// ============================================================
// JointState callback
// ============================================================
void BulldozerBladeControl::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lk(mtx_);

  const size_t n =
    std::min(
      msg->name.size(),
      msg->position.size());

  for (size_t i = 0; i < n; ++i)
  {
    if (std::isfinite(msg->position[i]))
    {
      latest_pos_[msg->name[i]] =
        msg->position[i];
    }
  }
}


// ============================================================
// publish_hold_command
//
// blade_hold=true のActionが成功した後も
// 同じJointCmdを送り続ける。
// ============================================================
void BulldozerBladeControl::publish_hold_command()
{
  std::lock_guard<std::mutex> lk(hold_mtx_);

  if (!blade_hold_active_) {
    return;
  }

  cmd_pub_->publish(hold_cmd_);
}


// ============================================================
// execute
// ============================================================
void BulldozerBladeControl::execute(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  auto result =
    std::make_shared<BladeAction::Result>();

  const auto goal =
    goal_handle->get_goal();

  const auto & joints =
    goal->joint_name;

  const auto & gp =
    goal->goal_position;

  const uint8_t ct =
    goal->control_type;

  const bool blade_hold =
    goal->blade_hold;

  const size_t n =
    joints.size();

  // ============================================================
  // 新しいActionが来たら、まず現在の保持状態を解除
  //
  // 特に blade_hold=false の場合、
  // 前回のhold_cmdが送られ続けないようにする。
  // ============================================================
  {
    std::lock_guard<std::mutex> lk(hold_mtx_);

    blade_hold_active_ = false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Start blade control: blade_hold=%s",
    blade_hold ? "true" : "false");

  // ============================================================
  // JointCmd
  // ============================================================
  com3_msgs::msg::JointCmd cmd;

  cmd.joint_name =
    joints;

  cmd.control_type =
    ct;

  cmd.position.assign(
    n,
    0.0);

  cmd.velocity.assign(
    n,
    0.0);

  cmd.effort.assign(
    n,
    0.0);

  if (ct == 0)
  {
    cmd.position =
      gp;
  }
  else if (ct == 1)
  {
    cmd.velocity =
      goal->velocity;
  }
  else
  {
    cmd.effort =
      goal->effort;
  }

  // ============================================================
  // 制御ループ
  // ============================================================
  rclcpp::Rate rate(loop_hz_);

  const auto start =
    this->now();

  while (rclcpp::ok())
  {
    // ----------------------------------------------------------
    // cancel
    // ----------------------------------------------------------
    if (goal_handle->is_canceling())
    {
      result->success = false;

      goal_handle->canceled(result);

      RCLCPP_INFO(
        this->get_logger(),
        "Blade control canceled.");

      return;
    }

    // ----------------------------------------------------------
    // publish command
    //
    // 目標到達前は通常通りここから送信する。
    // ----------------------------------------------------------
    cmd_pub_->publish(cmd);

    // ----------------------------------------------------------
    // error calculation
    // ----------------------------------------------------------
    std::vector<double> err(
      n,
      0.0);

    bool all_seen =
      true;

    {
      std::lock_guard<std::mutex> lk(mtx_);

      for (size_t i = 0; i < n; ++i)
      {
        auto it =
          latest_pos_.find(joints[i]);

        if (it == latest_pos_.end())
        {
          RCLCPP_WARN(
            this->get_logger(),
            "Joint NOT FOUND: %s",
            joints[i].c_str());

          all_seen = false;

          continue;
        }

        err[i] =
          gp[i] - it->second;

        RCLCPP_INFO(
          this->get_logger(),
          "joint=%s current=%.6f target=%.6f error=%.6f",
          joints[i].c_str(),
          it->second,
          gp[i],
          err[i]);
      }
    }

    // ----------------------------------------------------------
    // feedback
    // ----------------------------------------------------------
    auto fb =
      std::make_shared<BladeAction::Feedback>();

    fb->current_error =
      err;

    goal_handle->publish_feedback(fb);

    // ----------------------------------------------------------
    // 到達判定
    // ----------------------------------------------------------
    if (all_seen)
    {
      bool ok =
        true;

      for (size_t i = 0; i < n; ++i)
      {
        if (std::fabs(err[i]) > tolerance_)
        {
          ok = false;
          break;
        }
      }

      // ========================================================
      // 目標角に到達
      // ========================================================
      if (ok)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Blade target reached.");

        // ------------------------------------------------------
        // blade_hold=true
        //
        // 1. 現在のJointCmdをhold_cmd_として保存
        // 2. hold状態を有効化
        // 3. Action successを返す
        // 4. execute()終了後もTimerが送信し続ける
        // ------------------------------------------------------
        if (blade_hold)
        {
          {
            std::lock_guard<std::mutex> lk(hold_mtx_);

            hold_cmd_ =
              cmd;

            blade_hold_active_ =
              true;
          }

          RCLCPP_INFO(
            this->get_logger(),
            "Blade hold enabled. "
            "JointCmd will continue to be published.");

          result->success =
            true;

          goal_handle->succeed(result);

          return;
        }

        // ------------------------------------------------------
        // blade_hold=false
        //
        // 目標到達後にJointCmd送信を終了
        // ------------------------------------------------------
        else
        {
          {
            std::lock_guard<std::mutex> lk(hold_mtx_);

            blade_hold_active_ =
              false;
          }

          RCLCPP_INFO(
            this->get_logger(),
            "Blade target reached. "
            "Blade hold disabled. "
            "JointCmd publishing stopped.");

          result->success =
            true;

          goal_handle->succeed(result);

          return;
        }
      }
    }

    // ----------------------------------------------------------
    // timeout
    // ----------------------------------------------------------
    if ((this->now() - start).seconds()
        > timeout_sec_)
    {
      {
        std::lock_guard<std::mutex> lk(hold_mtx_);

        blade_hold_active_ =
          false;
      }

      result->success =
        false;

      goal_handle->abort(result);

      RCLCPP_WARN(
        this->get_logger(),
        "Blade control timeout.");

      return;
    }

    rate.sleep();
  }

  // ============================================================
  // ROS shutdown
  // ============================================================
  {
    std::lock_guard<std::mutex> lk(hold_mtx_);

    blade_hold_active_ =
      false;
  }

  if (goal_handle->is_active())
  {
    result->success =
      false;

    goal_handle->abort(result);
  }
}


// ============================================================
// main
// ============================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<BulldozerBladeControl>());

  rclcpp::shutdown();

  return 0;
}
