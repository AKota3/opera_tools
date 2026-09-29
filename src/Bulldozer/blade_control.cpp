#include "opera_tools/Bulldozer/blade_control.hpp"

#include <cmath>
#include <chrono>
#include <algorithm>

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


  control_type_ = this->declare_parameter<std::string>("command_interface_name", "position");

  RCLCPP_INFO(this->get_logger(), "command_interface_name = '%s'", control_type_.c_str());


  velocity_kp_ = this->declare_parameter<double>("velocity_kp", 1.0);

  effort_kp_ = this->declare_parameter<double>("effort_kp", 1.0);

  max_velocity_ = this->declare_parameter<double>("max_velocity", 1.0);

  max_effort_ = this->declare_parameter<double>("max_effort", 1.0);


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
  // command_interface_name から control_type を決定
  //
  // position : 0
  // velocity : 1
  // effort   : 2
  //
  // ※ Action Goalのcontrol_typeは使用しない
  // ------------------------------------------------------------
  control_type_ =
    this->get_parameter("command_interface_name").as_string();

  uint8_t control_type = 0;

  if (control_type_ == "velocity") {
    control_type = 1;
  }
  else if (control_type_ == "effort") {
    control_type = 2;
  }
  else {
    control_type = 0;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Received goal: command_interface=%s, control_type=%u",
    control_type_.c_str(),
    control_type);

  // ------------------------------------------------------------
  // joint_name
  // ------------------------------------------------------------
  const size_t n =
    goal->joint_name.size();

  if (n == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject: joint_name is empty");

    return rclcpp_action::GoalResponse::REJECT;
  }

  // ------------------------------------------------------------
  // goal_position
  //
  // すべてのcontrol_typeで必須。
  //
  // goal_positionは「最終的な目標角度」を表す。
  // ------------------------------------------------------------
  if (goal->goal_position.size() != n) {
    RCLCPP_WARN(
      this->get_logger(),
      "Reject: goal_position size mismatch "
      "(joint_name=%zu, goal_position=%zu)",
      n,
      goal->goal_position.size());

    return rclcpp_action::GoalResponse::REJECT;
  }

  // ------------------------------------------------------------
  // velocity / effort はチェックしない
  //
  // velocity controlの場合でも、
  // goal->velocity は入力として使用しない。
  //
  // effort controlの場合でも、
  // goal->effort は入力として使用しない。
  //
  // どちらもgoal_positionからexecute()内で計算する。
  // ------------------------------------------------------------

  // ------------------------------------------------------------
  // finite check
  //
  // goal_positionだけチェックすればよい
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
  }

  // ------------------------------------------------------------
  // blade_hold
  // ------------------------------------------------------------
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

  // const uint8_t ct =
  //   goal->control_type;

  uint8_t ct = 0;

  if (control_type_ == "velocity") {
    ct = 1;
  }
  else if (control_type_ == "effort") {
    ct = 2;
  }
  else if(control_type_ == "position") {
    ct = 0;
  }
  else {
  RCLCPP_ERROR(this->get_logger(), "Unknown command_interface_name: '%s'", control_type_.c_str());
  result->success = false;
  goal_handle->abort(result);
  return;
  }

  RCLCPP_INFO(this->get_logger(), "Using command_interface_name='%s', control_type=%u", control_type_.c_str(), ct);

  const bool blade_hold =
    goal->blade_hold;

  const size_t n =
    joints.size();

  // ============================================================
  // 新しいActionが来たら、まず現在の保持状態を解除
  // ============================================================
  {
    std::lock_guard<std::mutex> lk(hold_mtx_);

    blade_hold_active_ = false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Start blade control: blade_hold=%s, control_type=%u",
    blade_hold ? "true" : "false",
    ct);

  // ============================================================
  // JointCmd
  //
  // control_typeに応じて、
  // position / velocity / effort のどれかを
  // 制御ループ内で計算する。
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
      {
        std::lock_guard<std::mutex> lk(hold_mtx_);

        blade_hold_active_ = false;
      }

      result->success = false;

      goal_handle->canceled(result);

      RCLCPP_INFO(
        this->get_logger(),
        "Blade control canceled.");

      return;
    }

    // ==========================================================
    // 現在角度との差を計算
    //
    // goal_position は常に「目標角度」として使用する。
    // ==========================================================
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

        // ------------------------------------------------------
        // 目標角度 - 現在角度
        // ------------------------------------------------------
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

    // ==========================================================
    // control_typeに応じて制御入力を生成
    //
    // control_type:
    //
    //   0 : Position control
    //       position = goal_position
    //
    //   1 : Velocity control
    //       velocity = Kp * position_error
    //
    //   2 : Effort control
    //       effort = Kp * position_error
    //
    // goal_positionは全ての場合で目標角度として使用する。
    // ==========================================================

    // ----------------------------------------------------------
    // 一旦すべて0にする
    // ----------------------------------------------------------
    cmd.position.assign(
      n,
      0.0);

    cmd.velocity.assign(
      n,
      0.0);

    cmd.effort.assign(
      n,
      0.0);

    // ----------------------------------------------------------
    // Position control
    // ----------------------------------------------------------
    if (ct == 0)
    {
      for (size_t i = 0; i < n; ++i)
      {
        cmd.position[i] =
          gp[i];
      }
    }

    // ----------------------------------------------------------
    // Velocity control
    //
    // 目標角度との差から速度を生成する。
    // ----------------------------------------------------------
    else if (ct == 1)
    {
      for (size_t i = 0; i < n; ++i)
      {
        double velocity =
          velocity_kp_ * err[i];

        velocity =
          std::clamp(
            velocity,
            -max_velocity_,
            max_velocity_);

        cmd.velocity[i] =
          velocity;
      }
    }

    // ----------------------------------------------------------
    // Effort control
    //
    // 目標角度との差からeffortを生成する。
    // ----------------------------------------------------------
    else if (ct == 2)
    {
      for (size_t i = 0; i < n; ++i)
      {
        double effort =
          effort_kp_ * err[i];

        effort =
          std::clamp(
            effort,
            -max_effort_,
            max_effort_);

        cmd.effort[i] =
          effort;
      }
    }

    // ----------------------------------------------------------
    // 不正なcontrol_type
    // ----------------------------------------------------------
    else
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid control_type=%u",
        ct);

      {
        std::lock_guard<std::mutex> lk(hold_mtx_);

        blade_hold_active_ = false;
      }

      result->success =
        false;

      goal_handle->abort(result);

      return;
    }

    // ==========================================================
    // JointCmdをpublish
    //
    // 目標到達前は、毎周期ここで制御入力を更新して送信する。
    // ==========================================================
    cmd_pub_->publish(cmd);

    // ==========================================================
    // feedback
    // ==========================================================
    auto fb =
      std::make_shared<BladeAction::Feedback>();

    fb->current_error =
      err;

    goal_handle->publish_feedback(fb);

    // ==========================================================
    // 到達判定
    // ==========================================================
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
      // 目標角度に到達
      // ========================================================
      if (ok)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Blade target reached.");

        // ======================================================
        // blade_hold=true
        //
        // command_interface_name に応じて保持コマンドを設定する。
        //
        // position:
        //   goal_position をそのまま送信し続ける。
        //
        // velocity:
        //   0, 0, 0 を送信し続ける。
        //
        // effort:
        //   0, 0, 0 を送信し続ける。
        // ======================================================
        if (blade_hold)
        {
          {
            std::lock_guard<std::mutex> lk(hold_mtx_);

            hold_cmd_ = cmd;

            if (ct == 0)
            {
              // --------------------------------------------------
              // position control
              // --------------------------------------------------
              // 目標姿勢をそのまま保持する
              hold_cmd_.position = gp;

              // velocity / effort は0
              hold_cmd_.velocity.assign(n, 0.0);
              hold_cmd_.effort.assign(n, 0.0);
            }
            else if (ct == 1)
            {
              // --------------------------------------------------
              // velocity control
              // --------------------------------------------------
              // 目標到達後は速度0で停止する
              hold_cmd_.position.assign(n, 0.0);
              hold_cmd_.velocity.assign(n, 0.0);
              hold_cmd_.effort.assign(n, 0.0);
            }
            else if (ct == 2)
            {
              // --------------------------------------------------
              // effort control
              // --------------------------------------------------
              // 目標到達後はeffort 0にする
              hold_cmd_.position.assign(n, 0.0);
              hold_cmd_.velocity.assign(n, 0.0);
              hold_cmd_.effort.assign(n, 0.0);
            }

            // control_typeは現在の制御方式を維持
            hold_cmd_.control_type = ct;

            // joint_nameも維持
            hold_cmd_.joint_name = joints;

            blade_hold_active_ = true;
          }

          RCLCPP_INFO(
            this->get_logger(),
            "Blade hold enabled: interface=%s, control_type=%u",
            control_type_.c_str(),
            static_cast<unsigned int>(ct));

          result->success = true;

          goal_handle->succeed(result);

          return;
        }


        // ======================================================
        // blade_hold=false
        //
        // 目標到達後、JointCmdの送信を停止する。
        // ======================================================
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

    // ==========================================================
    // timeout
    // ==========================================================
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
