// Copyright 2026 Ian

#include <gpiod.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <fstream>
#include <iterator>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "kilin_msgs/msg/balance_state_stamped.hpp"
#include "kilin_msgs/msg/leg_cmd.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/stability_state_stamped.hpp"
#include "kilin_msgs/msg/stair_phase_stamped.hpp"
#include "kilin_msgs/msg/trigger_stamped.hpp"
#include "kilin_stair_controller/support_geometry.hpp"
#include "kinova_ptp_interfaces/action/joint_ptp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{
std::atomic_bool g_sigint_requested{false};

void sigint_handler([[maybe_unused]] int signal_number)
{
  g_sigint_requested.store(true);
}

struct GaitPoint
{
  double time{};
  double a_hip_pos{}, a_steer_pos{}, a_hub_vel{}, a_hub_mode{};
  double b_hip_pos{}, b_steer_pos{}, b_hub_vel{}, b_hub_mode{};
  double c_hip_pos{}, c_steer_pos{}, c_hub_vel{}, c_hub_mode{};
  double d_hip_pos{}, d_steer_pos{}, d_hub_vel{}, d_hub_mode{};
  int arm_phase{0};
};
}  // namespace

class KilinStairController : public rclcpp::Node
{
public:
  using JointPtp = kinova_ptp_interfaces::action::JointPtp;
  using JointPtpGoalHandle = rclcpp_action::ClientGoalHandle<JointPtp>;

  KilinStairController()
  : Node("kilin_stair_controller")
  {
    declare_parameter<std::string>("csv_path", "stairs.csv");
    declare_parameter<double>("rate_hz", 100.0);
    declare_parameter<double>("delay_start_sec", 3.0);
    declare_parameter<bool>("arm_enabled", true);
    declare_parameter<double>("arm_timeout_sec", 18.0);
    declare_parameter<double>("arm_ptp_duration_sec", 3.0);
    declare_parameter<std::string>("arm_action_name", "/kinova_joint_ptp");
    declare_parameter<std::string>("arm_control_mode", "fixed_phase");
    declare_parameter<std::string>("balance_state_topic", "/kilin/balance_state");
    declare_parameter<std::string>("stability_state_topic", "/kilin/stability_state");
    declare_parameter<double>("balance_state_timeout_sec", 0.5);
    declare_parameter<double>("com_safe_margin_m", 0.01);
    declare_parameter<double>("com_alpha_step", 0.05);
    declare_parameter<double>("com_safe_hold_sec", 0.3);
    declare_parameter<double>("amr_yaw_in_world_deg", 0.0);
    const std::vector<double> default_standard_pose =
    {0.0, -85.94, 0.0, 147.0, 0.0, 22.92, 0.0};
    const std::vector<double> default_front_pose =
    {0.0, 20.054, 0.0, -88.808, 0.0, 63.025, 0.0};
    const std::vector<double> default_rear_pose =
    {0.0, -11.459, 0.0, -68.755, 0.0, 45.837, 0.0};
    const std::vector<double> default_full_extension_pose =
    {0.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    declare_parameter<std::vector<double>>("front_left_pose_deg", default_front_pose);
    declare_parameter<std::vector<double>>("front_right_pose_deg", default_front_pose);
    declare_parameter<std::vector<double>>("rear_left_pose_deg", default_rear_pose);
    declare_parameter<std::vector<double>>("rear_right_pose_deg", default_rear_pose);
    declare_parameter<std::vector<double>>("standard_pose_deg", default_standard_pose);
    declare_parameter<std::vector<double>>(
      "full_extension_pose_deg", default_full_extension_pose);
    declare_parameter<bool>("invert_ab_hips_on_hardware", true);
    declare_parameter<std::string>("trigger_chip", "/dev/gpiochip0");
    declare_parameter<int>("trigger_line", 112);

    csv_path_ = get_parameter("csv_path").as_string();
    rate_hz_ = get_parameter("rate_hz").as_double();
    delay_start_sec_ = get_parameter("delay_start_sec").as_double();
    arm_enabled_ = get_parameter("arm_enabled").as_bool();
    arm_timeout_sec_ = get_parameter("arm_timeout_sec").as_double();
    arm_ptp_duration_sec_ = get_parameter("arm_ptp_duration_sec").as_double();
    arm_action_name_ = get_parameter("arm_action_name").as_string();
    arm_control_mode_ = get_parameter("arm_control_mode").as_string();
    closed_loop_arm_ = arm_control_mode_ == "com_closed_loop";
    balance_state_topic_ = get_parameter("balance_state_topic").as_string();
    stability_state_topic_ = get_parameter("stability_state_topic").as_string();
    balance_state_timeout_sec_ = get_parameter("balance_state_timeout_sec").as_double();
    com_safe_margin_m_ = get_parameter("com_safe_margin_m").as_double();
    com_alpha_step_ = get_parameter("com_alpha_step").as_double();
    com_safe_hold_sec_ = get_parameter("com_safe_hold_sec").as_double();
    amr_yaw_in_world_rad_ = get_parameter("amr_yaw_in_world_deg").as_double() * M_PI / 180.0;
    standard_pose_ =
      degrees_to_radians(get_parameter("standard_pose_deg").as_double_array());
    full_extension_pose_ =
      degrees_to_radians(get_parameter("full_extension_pose_deg").as_double_array());
    arm_poses_[0] =
      degrees_to_radians(get_parameter("front_left_pose_deg").as_double_array());
    arm_poses_[1] =
      degrees_to_radians(get_parameter("front_right_pose_deg").as_double_array());
    arm_poses_[2] =
      degrees_to_radians(get_parameter("rear_left_pose_deg").as_double_array());
    arm_poses_[3] =
      degrees_to_radians(get_parameter("rear_right_pose_deg").as_double_array());
    held_arm_pose_ = standard_pose_;
    const bool use_sim_time = get_parameter("use_sim_time").as_bool();
    invert_ab_hips_ =
      !use_sim_time && get_parameter("invert_ab_hips_on_hardware").as_bool();
    trigger_chipname_ = get_parameter("trigger_chip").as_string();
    trigger_line_offset_ = static_cast<unsigned int>(get_parameter("trigger_line").as_int());

    const bool invalid_compensation_pose = std::any_of(
      arm_poses_.begin(), arm_poses_.end(),
      [](const auto & pose) {return pose.size() != 7;});
    if (rate_hz_ <= 0.0 || arm_timeout_sec_ <= 0.0 || arm_ptp_duration_sec_ <= 0.0 ||
      arm_timeout_sec_ <= arm_ptp_duration_sec_ || arm_action_name_.empty() ||
      standard_pose_.size() != 7 || full_extension_pose_.size() != 7 ||
      invalid_compensation_pose ||
      (arm_control_mode_ != "fixed_phase" && arm_control_mode_ != "com_closed_loop") ||
      balance_state_timeout_sec_ <= 0.0 || com_safe_margin_m_ < 0.0 ||
      com_alpha_step_ <= 0.0 || com_alpha_step_ > 1.0 || com_safe_hold_sec_ < 0.0)
    {
      throw std::runtime_error(
              "Invalid rate/action/timing parameter or Kinova pose; arm_timeout_sec must "
              "exceed arm_ptp_duration_sec, poses must contain 7 values, and COM parameters "
              "must be in range");
    }
    if (!load_csv(csv_path_)) {
      throw std::runtime_error("Failed to load CSV file: " + csv_path_);
    }

    motor_pub_ = create_publisher<kilin_msgs::msg::MotorCmdStamped>("/kilin/motor_cmd_raw", 10);
    arm_client_ = rclcpp_action::create_client<JointPtp>(this, arm_action_name_);
    auto trigger_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    trigger_pub_ = create_publisher<kilin_msgs::msg::TriggerStamped>("/kilin/trigger", trigger_qos);
    phase_pub_ = create_publisher<kilin_msgs::msg::StairPhaseStamped>(
      "/kilin/stair_phase", trigger_qos);
    stability_pub_ = create_publisher<kilin_msgs::msg::StabilityStateStamped>(
      stability_state_topic_, 10);
    balance_sub_ = create_subscription<kilin_msgs::msg::BalanceStateStamped>(
      balance_state_topic_, 10,
      std::bind(&KilinStairController::balance_state_callback, this, std::placeholders::_1));

    start_time_ = get_clock()->now();
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::microseconds(static_cast<int64_t>(1e6 / rate_hz_)),
      std::bind(&KilinStairController::update, this));

    RCLCPP_INFO(
      get_logger(), "Loaded %zu stair gait points from %s", gait_.size(),
      csv_path_.c_str());
    RCLCPP_INFO(
      get_logger(), "Mode uses %s time. Arm phases: %s. Playback starts in %.1f seconds.",
      use_sim_time ? "simulation" : "system",
      csv_has_arm_phase_ && arm_enabled_ ? "enabled" : "disabled", delay_start_sec_);
    RCLCPP_INFO(
      get_logger(), "Module A/B hip inversion: %s", invert_ab_hips_ ? "enabled" : "disabled");
    RCLCPP_INFO(
      get_logger(), "Kinova PTP action: %s, duration: %.2f s, timeout: %.2f s",
      arm_action_name_.c_str(), arm_ptp_duration_sec_, arm_timeout_sec_);
    RCLCPP_INFO(
      get_logger(), "Arm control mode: %s", arm_control_mode_.c_str());
    if (closed_loop_arm_) {
      RCLCPP_INFO(
        get_logger(),
        "COM input: %s, safe margin: %.1f mm, alpha step: %.3f, AMR yaw: %.2f deg",
        balance_state_topic_.c_str(), com_safe_margin_m_ * 1000.0, com_alpha_step_,
        amr_yaw_in_world_rad_ * 180.0 / M_PI);
    }
  }

  ~KilinStairController() override
  {
    cancel_active_arm_goal();
    trigger_off();
    cleanup_trigger_gpio();
  }

private:
  bool load_csv(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      return false;
    }

    std::string line;
    if (!std::getline(file, line)) {
      RCLCPP_ERROR(get_logger(), "CSV is empty.");
      return false;
    }
    csv_has_arm_phase_ = line.find("arm_phase") != std::string::npos;

    std::size_t line_number = 1;
    while (std::getline(file, line)) {
      ++line_number;
      if (line.empty()) {
        continue;
      }
      try {
        std::stringstream stream(line);
        std::string cell;
        std::vector<double> values;
        while (std::getline(stream, cell, ',')) {
          values.push_back(std::stod(cell));
        }

        const std::size_t data_columns = values.size() - (csv_has_arm_phase_ ? 1U : 0U);
        if (data_columns != 13 && data_columns != 17) {
          RCLCPP_ERROR(
            get_logger(), "CSV line %zu has %zu gait columns; expected 13 or 17.",
            line_number, data_columns);
          return false;
        }

        const bool has_steering = data_columns == 17;
        std::size_t k = 0;
        GaitPoint point;
        point.time = values[k++];
        parse_module(
          values, k, has_steering, point.a_hip_pos, point.a_steer_pos,
          point.a_hub_vel, point.a_hub_mode);
        parse_module(
          values, k, has_steering, point.b_hip_pos, point.b_steer_pos,
          point.b_hub_vel, point.b_hub_mode);
        parse_module(
          values, k, has_steering, point.c_hip_pos, point.c_steer_pos,
          point.c_hub_vel, point.c_hub_mode);
        parse_module(
          values, k, has_steering, point.d_hip_pos, point.d_steer_pos,
          point.d_hub_vel, point.d_hub_mode);
        if (csv_has_arm_phase_) {
          point.arm_phase = static_cast<int>(std::lround(values[k]));
          if (point.arm_phase < 0 || point.arm_phase > 4) {
            RCLCPP_ERROR(
              get_logger(), "CSV line %zu has invalid arm_phase %d.", line_number, point.arm_phase);
            return false;
          }
        }
        gait_.push_back(point);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "CSV parse error on line %zu: %s", line_number, error.what());
        return false;
      }
    }

    if (gait_.empty()) {
      RCLCPP_ERROR(get_logger(), "CSV contains no gait points.");
      return false;
    }
    if (!std::is_sorted(
        gait_.begin(), gait_.end(), [](const auto & lhs, const auto & rhs) {
          return lhs.time < rhs.time;
        }))
    {
      RCLCPP_ERROR(get_logger(), "CSV timestamps must be sorted.");
      return false;
    }
    return true;
  }

  static void parse_module(
    const std::vector<double> & values, std::size_t & k, bool has_steering,
    double & hip, double & steering, double & hub_velocity, double & hub_mode)
  {
    hip = values[k++];
    steering = has_steering ? values[k++] : 0.0;
    hub_velocity = values[k++];
    hub_mode = values[k++];
  }

  static std::vector<double> degrees_to_radians(const std::vector<double> & degrees)
  {
    std::vector<double> radians;
    radians.reserve(degrees.size());
    std::transform(
      degrees.begin(), degrees.end(), std::back_inserter(radians),
      [](double degree) {return degree * M_PI / 180.0;});
    return radians;
  }

  void update()
  {
    const auto now = get_clock()->now();
    if (g_sigint_requested.load()) {
      finish("SIGINT");
      return;
    }
    if (now.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for a valid clock.");
      return;
    }
    if (!started_) {
      if ((now - start_time_).seconds() < delay_start_sec_) {
        return;
      }
      playback_start_time_ = now;
      started_ = true;
      trigger_on();
      publish_trigger(true);
      RCLCPP_INFO(get_logger(), "Stair playback started.");
    }

    if (waiting_for_arm_) {
      if (last_motor_command_valid_) {
        last_motor_command_.header.time.sec = static_cast<int32_t>(now.seconds());
        last_motor_command_.header.time.nanosec = now.nanoseconds() % 1000000000;
        motor_pub_->publish(last_motor_command_);
      }
      if (closed_loop_arm_ && active_arm_phase_ > 0 && arm_waypoints_.empty()) {
        prepare_closed_loop_waypoints();
        if (arm_waypoints_.empty() && !arm_reached_ &&
          (now - arm_wait_start_time_).seconds() > arm_timeout_sec_)
        {
          arm_request_failed_ = true;
          arm_failure_message_ = "timed out waiting for a valid balance state";
        }
      }
      const bool should_send_arm_goal =
        !arm_goal_sent_ && !arm_reached_ && !arm_request_failed_ && !arm_waypoints_.empty();
      if (safe_hold_pending_) {
        begin_or_update_safe_hold(now);
      } else if (should_send_arm_goal) {
        try_send_arm_goal();
      }
      const bool arm_waypoint_timed_out =
        !arm_waypoints_.empty() &&
        (now - arm_goal_wait_start_time_).seconds() > arm_timeout_sec_;
      if (arm_request_failed_) {
        RCLCPP_ERROR(
          get_logger(), "Kinova arm phase %d failed: %s", active_arm_phase_,
          arm_failure_message_.c_str());
        finish("arm action failed");
      } else if (arm_reached_) {
        playback_start_time_ = playback_start_time_ + (now - arm_wait_start_time_);
        waiting_for_arm_ = false;
        arm_reached_ = false;
        arm_goal_sent_ = false;
        publish_stair_phase("complete", false);
        RCLCPP_INFO(get_logger(), "Kinova reached arm phase %d; resuming gait.", active_arm_phase_);
      } else if (arm_waypoint_timed_out) {
        RCLCPP_ERROR(
          get_logger(), "Kinova phase %d waypoint %zu/%zu (%s) timed out after %.1f seconds.",
          active_arm_phase_, active_arm_waypoint_ + 1, arm_waypoints_.size(),
          current_arm_waypoint().name.c_str(), arm_timeout_sec_);
        cancel_active_arm_goal();
        finish("arm timeout");
      }
      return;
    }

    const double t = (now - playback_start_time_).seconds();
    const bool playback_at_end = t >= gait_.back().time;
    const double sample_time = std::min(t, gait_.back().time);

    const auto upper = std::upper_bound(
      gait_.begin(), gait_.end(), sample_time,
      [](double time, const GaitPoint & point) {return time < point.time;});
    const std::size_t p0_index =
      upper ==
      gait_.begin() ? 0U : static_cast<std::size_t>(std::distance(gait_.begin(), upper) - 1);
    const std::size_t p1_index = std::min(p0_index + 1, gait_.size() - 1);
    const auto & p0 = gait_[p0_index];
    const auto & p1 = gait_[p1_index];

    const bool arm_control_enabled = arm_enabled_ && csv_has_arm_phase_;
    if (arm_control_enabled) {
      if (active_arm_phase_ < 0) {
        // Establish the first row exactly before commanding its arm pose.
        publish_motor_command(build_motor_command(p0.time, p0, p0));
        request_arm_phase(p0.arm_phase, now);
        return;
      }

      if (!playback_at_end && p1.arm_phase != active_arm_phase_) {
        // Every phase transition belongs to the motion arriving at p1,
        // including transitions back to phase 0. Keep the entire last Kilin
        // command unchanged until Kinova reaches the requested pose; only then
        // allow hip, steering, and wheel commands to advance from p0 to p1.
        if (!last_motor_command_valid_) {
          publish_motor_command(build_motor_command(p0.time, p0, p0));
        }
        request_arm_phase(p1.arm_phase, now);
        return;
      }
    }

    publish_motor_command(build_motor_command(sample_time, p0, p1));
    if (playback_at_end) {
      finish("playback complete");
    }
  }

  void publish_motor_command(const kilin_msgs::msg::MotorCmdStamped & command)
  {
    motor_pub_->publish(command);
    last_motor_command_ = command;
    last_motor_command_valid_ = true;
  }

  kilin_msgs::msg::MotorCmdStamped build_motor_command(
    double t, const GaitPoint & p0, const GaitPoint & p1)
  {
    kilin_msgs::msg::MotorCmdStamped msg;
    fill_header(msg);
    msg.module_a = build_leg(
      t, p0, p1, p0.a_hip_pos, p1.a_hip_pos,
      p0.a_steer_pos, p1.a_steer_pos, p1.a_hub_vel, p1.a_hub_mode, invert_ab_hips_);
    msg.module_b = build_leg(
      t, p0, p1, p0.b_hip_pos, p1.b_hip_pos,
      p0.b_steer_pos, p1.b_steer_pos, p1.b_hub_vel, p1.b_hub_mode, invert_ab_hips_);
    msg.module_c = build_leg(
      t, p0, p1, p0.c_hip_pos, p1.c_hip_pos,
      p0.c_steer_pos, p1.c_steer_pos, p1.c_hub_vel, p1.c_hub_mode, false);
    msg.module_d = build_leg(
      t, p0, p1, p0.d_hip_pos, p1.d_hip_pos,
      p0.d_steer_pos, p1.d_steer_pos, p1.d_hub_vel, p1.d_hub_mode, false);
    return msg;
  }

  static double lerp(double t, double t0, double t1, double v0, double v1)
  {
    return t1 == t0 ? v0 : v0 + ((t - t0) / (t1 - t0)) * (v1 - v0);
  }

  static kilin_msgs::msg::LegCmd build_leg(
    double t, const GaitPoint & p0, const GaitPoint & p1, double hip0, double hip1,
    double steer0, double steer1, double hub_velocity, double hub_mode, bool invert_hip)
  {
    kilin_msgs::msg::LegCmd leg;
    double hip_deg = lerp(t, p0.time, p1.time, hip0, hip1);
    if (invert_hip) {
      hip_deg = -hip_deg;
    }
    leg.hip.position = hip_deg * M_PI / 180.0;
    leg.hip.motor_mode = 4;
    leg.hip.kp = 350.0;
    leg.hip.ki = 0.0;
    leg.hip.kd = 5.0;
    leg.steering.position = lerp(t, p0.time, p1.time, steer0, steer1) * M_PI / 180.0;
    leg.steering.motor_mode = 4;
    leg.hub.velocity = hub_velocity;
    leg.hub.motor_mode = static_cast<int>(hub_mode);
    return leg;
  }

  void request_arm_phase(int phase, const rclcpp::Time & now)
  {
    const int previous_phase = active_arm_phase_;
    arm_waypoints_.clear();
    safe_hold_pending_ = false;

    active_arm_waypoint_ = 0;
    active_arm_phase_ = phase;
    previous_arm_phase_ = previous_phase;
    closed_loop_standard_pending_ = closed_loop_arm_ && phase > 0 && previous_phase != 0;
    closed_loop_waiting_for_standard_ = false;
    waiting_for_arm_ = true;
    arm_reached_ = false;
    arm_goal_sent_ = false;
    arm_request_failed_ = false;
    arm_failure_message_.clear();
    active_arm_goal_.reset();
    arm_wait_start_time_ = now;
    arm_goal_wait_start_time_ = now;

    if (closed_loop_arm_ && phase > 0) {
      if (!prepare_closed_loop_waypoints()) {
        publish_stair_phase("waiting-balance-state", true);
        RCLCPP_INFO(
          get_logger(),
          "Requested geometry COM adjustment for phase %d (%s leg moving); gait paused.",
          phase, arm_phase_name(phase));
        return;
      }
    } else {
      build_fixed_or_standard_waypoints(phase, previous_phase);
    }

    if (arm_reached_) {
      publish_stair_phase("already-safe", true);
      return;
    }
    pending_arm_target_ = arm_waypoints_.front().positions;
    arm_goal_wait_start_time_ = get_clock()->now();
    publish_stair_phase(current_arm_waypoint().name, true);
    try_send_arm_goal();
    if (phase == 0) {
      RCLCPP_INFO(
        get_logger(), "Requested Kinova standard pose using %zu waypoint(s); gait paused.",
        arm_waypoints_.size());
    } else if (closed_loop_arm_) {
      RCLCPP_INFO(
        get_logger(),
        "Requested closed-loop COM adjustment for phase %d using at most %zu waypoint(s).",
        phase, arm_waypoints_.size());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Requested Kinova arm phase %d (%s leg moving) using %zu waypoint(s); gait paused.",
        phase, arm_phase_name(phase), arm_waypoints_.size());
    }
  }

  void build_fixed_or_standard_waypoints(int phase, int previous_phase)
  {
    if (phase == 0) {
      if (previous_phase > 0) {
        auto retracted_pose = standard_pose_;
        retracted_pose[0] = held_arm_pose_[0];
        arm_waypoints_.push_back({"retract", std::move(retracted_pose), -1.0});
      }
      arm_waypoints_.push_back({"rotate-home", standard_pose_, -1.0});
    } else {
      if (previous_phase > 0) {
        auto retracted_pose = standard_pose_;
        retracted_pose[0] = held_arm_pose_[0];
        arm_waypoints_.push_back({"retract", std::move(retracted_pose), -1.0});
      } else if (previous_phase < 0) {
        arm_waypoints_.push_back({"establish-standard", standard_pose_, -1.0});
      }

      const auto & final_pose = arm_poses_.at(static_cast<std::size_t>(phase - 1));
      auto rotated_pose = standard_pose_;
      rotated_pose[0] = final_pose[0];
      arm_waypoints_.push_back({"rotate-joint-1", std::move(rotated_pose), -1.0});
      arm_waypoints_.push_back({"extend", final_pose, -1.0});
    }
  }

  bool prepare_closed_loop_waypoints()
  {
    if (!arm_waypoints_.empty() || arm_reached_) {
      return true;
    }
    if (closed_loop_standard_pending_) {
      if (previous_arm_phase_ > 0) {
        auto retracted_pose = standard_pose_;
        retracted_pose[0] = held_arm_pose_[0];
        arm_waypoints_.push_back({"retract-before-com", std::move(retracted_pose), -1.0});
      } else {
        arm_waypoints_.push_back({"establish-standard-before-com", standard_pose_, -1.0});
      }
      closed_loop_standard_pending_ = false;
      closed_loop_waiting_for_standard_ = true;
      pending_arm_target_ = arm_waypoints_.front().positions;
      arm_goal_wait_start_time_ = get_clock()->now();
      publish_stair_phase(current_arm_waypoint().name, true);
      return true;
    }
    const auto stability = latest_stability(true);
    if (!stability) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for a recent, finite %s sample before phase %d COM adjustment.",
        balance_state_topic_.c_str(), active_arm_phase_);
      return false;
    }
    if (stability->inside_safe_region) {
      arm_reached_ = true;
      RCLCPP_INFO(
        get_logger(), "Phase %d is already safe: margin %.3f mm.", active_arm_phase_,
        stability->signed_margin * 1000.0);
      return true;
    }

    const double cosine = std::cos(amr_yaw_in_world_rad_);
    const double sine = std::sin(amr_yaw_in_world_rad_);
    const double direction_amr_x =
      cosine * stability->direction.x + sine * stability->direction.y;
    const double direction_amr_y =
      -sine * stability->direction.x + cosine * stability->direction.y;
    double target_j1 = -std::atan2(direction_amr_y, direction_amr_x);
    const double reference_j1 = held_arm_pose_[0];
    target_j1 = reference_j1 + std::remainder(target_j1 - reference_j1, 2.0 * M_PI);

    auto rotated_pose = standard_pose_;
    rotated_pose[0] = target_j1;
    arm_waypoints_.push_back({"rotate-joint-1", std::move(rotated_pose), 0.0});
    for (double alpha = com_alpha_step_; alpha < 1.0 - 1e-9; alpha += com_alpha_step_) {
      arm_waypoints_.push_back(
        {"extend-alpha-" + std::to_string(alpha), extension_pose(target_j1, alpha), alpha});
    }
    arm_waypoints_.push_back({"extend-alpha-1.000000", extension_pose(target_j1, 1.0), 1.0});
    pending_arm_target_ = arm_waypoints_.front().positions;
    publish_stair_phase(current_arm_waypoint().name, true);
    RCLCPP_INFO(
      get_logger(),
      "Phase %d initial margin %.3f mm; correction %.3f mm, world direction "
      "[%.4f, %.4f], J1 %.3f deg.",
      active_arm_phase_, stability->signed_margin * 1000.0,
      stability->correction_distance * 1000.0, stability->direction.x,
      stability->direction.y, target_j1 * 180.0 / M_PI);
    return true;
  }

  std::vector<double> extension_pose(double target_j1, double alpha) const
  {
    std::vector<double> pose(7);
    for (std::size_t i = 0; i < pose.size(); ++i) {
      pose[i] = standard_pose_[i] + alpha * (full_extension_pose_[i] - standard_pose_[i]);
    }
    pose[0] = target_j1;
    return pose;
  }

  std::optional<kilin_stair_controller::geometry::StabilityResult> latest_stability(
    bool require_recent)
  {
    if (!have_balance_state_) {
      return std::nullopt;
    }
    if (require_recent &&
      (get_clock()->now() - last_balance_receive_time_).seconds() > balance_state_timeout_sec_)
    {
      return std::nullopt;
    }
    try {
      std::array<kilin_stair_controller::geometry::Point2, 4> wheels;
      for (std::size_t i = 0; i < wheels.size(); ++i) {
        wheels[i] = {latest_balance_state_.contact_points[i].x,
          latest_balance_state_.contact_points[i].y};
      }
      return kilin_stair_controller::geometry::evaluate_stability(
        {latest_balance_state_.com.x, latest_balance_state_.com.y}, wheels,
        active_arm_phase_, com_safe_margin_m_);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Invalid balance geometry: %s", error.what());
      return std::nullopt;
    }
  }

  void balance_state_callback(const kilin_msgs::msg::BalanceStateStamped::SharedPtr msg)
  {
    latest_balance_state_ = *msg;
    last_balance_receive_time_ = get_clock()->now();
    have_balance_state_ = true;
    if (!closed_loop_arm_ || active_arm_phase_ < 1 || active_arm_phase_ > 4) {
      return;
    }
    const auto stability = latest_stability(false);
    if (stability) {
      publish_stability(*stability);
    }
  }

  void publish_stability(
    const kilin_stair_controller::geometry::StabilityResult & stability)
  {
    kilin_msgs::msg::StabilityStateStamped msg;
    msg.header = latest_balance_state_.header;
    msg.phase = active_arm_phase_;
    msg.swing_leg = arm_phase_name(active_arm_phase_);
    msg.valid = true;
    msg.inside = stability.inside_support;
    msg.safe = stability.inside_safe_region;
    msg.stability_margin = stability.signed_margin;
    msg.required_margin = com_safe_margin_m_;
    msg.com_projection = latest_balance_state_.com;
    for (const auto & point : stability.hull) {
      geometry_msgs::msg::Point32 output;
      output.x = static_cast<float>(point.x);
      output.y = static_cast<float>(point.y);
      output.z = static_cast<float>(latest_balance_state_.com.z);
      msg.support_polygon.points.push_back(output);
    }
    msg.correction_direction.x = stability.direction.x;
    msg.correction_direction.y = stability.direction.y;
    stability_pub_->publish(msg);
  }

  void begin_or_update_safe_hold(const rclcpp::Time & now)
  {
    const auto stability = latest_stability(true);
    if (!stability) {
      safe_hold_pending_ = false;
      arm_request_failed_ = true;
      arm_failure_message_ = "balance state missing or stale during safe hold";
      return;
    }
    if (!stability->inside_safe_region) {
      safe_hold_pending_ = false;
      advance_closed_loop_waypoint();
      return;
    }
    if ((now - safe_hold_start_time_).seconds() < com_safe_hold_sec_) {
      return;
    }
    safe_hold_pending_ = false;
    arm_reached_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Phase %d COM adjustment succeeded: margin %.3f mm, alpha %.3f; safe for %.2f s.",
      active_arm_phase_, stability->signed_margin * 1000.0,
      current_arm_waypoint().alpha, com_safe_hold_sec_);
  }

  void advance_closed_loop_waypoint()
  {
    if (active_arm_waypoint_ + 1 >= arm_waypoints_.size()) {
      arm_request_failed_ = true;
      arm_failure_message_ = "maximum arm extension reached before COM entered safe region";
      return;
    }
    ++active_arm_waypoint_;
    pending_arm_target_ = current_arm_waypoint().positions;
    arm_goal_sent_ = false;
    arm_goal_wait_start_time_ = get_clock()->now();
    publish_stair_phase(current_arm_waypoint().name, true);
    RCLCPP_INFO(
      get_logger(), "COM remains outside safe region; advancing to alpha %.3f.",
      current_arm_waypoint().alpha);
  }

  void try_send_arm_goal()
  {
    if (arm_goal_sent_ || !waiting_for_arm_) {
      return;
    }
    if (!arm_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for Kinova PTP action server: %s",
        arm_action_name_.c_str());
      return;
    }

    JointPtp::Goal goal;
    goal.joint_names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
    goal.positions = pending_arm_target_;
    goal.duration_sec = arm_ptp_duration_sec_;

    auto options = rclcpp_action::Client<JointPtp>::SendGoalOptions();
    options.goal_response_callback =
      [this](const JointPtpGoalHandle::SharedPtr goal_handle) {
        if (!waiting_for_arm_) {
          return;
        }
        if (!goal_handle) {
          arm_request_failed_ = true;
          arm_failure_message_ = "JointPtp server rejected the goal";
          return;
        }
        active_arm_goal_ = goal_handle;
        RCLCPP_INFO(
          get_logger(), "Kinova accepted phase %d waypoint %zu/%zu (%s).",
          active_arm_phase_, active_arm_waypoint_ + 1, arm_waypoints_.size(),
          current_arm_waypoint().name.c_str());
      };
    options.feedback_callback =
      [this](
      JointPtpGoalHandle::SharedPtr,
      const std::shared_ptr<const JointPtp::Feedback> feedback) {
        if (!waiting_for_arm_) {
          return;
        }
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Kinova phase %d waypoint %zu/%zu (%s): %.1f%%, max error %.6f rad, state=%s",
          active_arm_phase_, active_arm_waypoint_ + 1, arm_waypoints_.size(),
          current_arm_waypoint().name.c_str(), feedback->progress * 100.0,
          feedback->max_joint_error, feedback->state.c_str());
      };
    options.result_callback =
      [this](const JointPtpGoalHandle::WrappedResult & wrapped_result) {
        active_arm_goal_.reset();
        if (!waiting_for_arm_) {
          return;
        }
        if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          arm_request_failed_ = true;
          arm_failure_message_ =
            wrapped_result.result && !wrapped_result.result->message.empty() ?
            wrapped_result.result->message :
            "JointPtp action did not finish with SUCCEEDED status";
          return;
        }
        if (!wrapped_result.result || !wrapped_result.result->success ||
          wrapped_result.result->error_code != JointPtp::Result::SUCCESS)
        {
          arm_request_failed_ = true;
          arm_failure_message_ = wrapped_result.result ?
            wrapped_result.result->message : "JointPtp returned no result";
          return;
        }
        RCLCPP_INFO(
          get_logger(), "Kinova phase %d waypoint %zu/%zu (%s) succeeded; error %.6f rad.",
          active_arm_phase_, active_arm_waypoint_ + 1, arm_waypoints_.size(),
          current_arm_waypoint().name.c_str(),
          wrapped_result.result->final_max_joint_error);
        held_arm_pose_ = current_arm_waypoint().positions;

        if (closed_loop_arm_ && active_arm_phase_ > 0 &&
          closed_loop_waiting_for_standard_ && current_arm_waypoint().alpha < 0.0)
        {
          closed_loop_waiting_for_standard_ = false;
          arm_waypoints_.clear();
          active_arm_waypoint_ = 0;
          arm_goal_sent_ = false;
          arm_goal_wait_start_time_ = get_clock()->now();
          RCLCPP_INFO(
            get_logger(), "Standard shape reached; recomputing phase %d COM correction.",
            active_arm_phase_);
          prepare_closed_loop_waypoints();
          return;
        }

        if (closed_loop_arm_ && active_arm_phase_ > 0 && current_arm_waypoint().alpha >= 0.0) {
          const auto stability = latest_stability(true);
          if (!stability) {
            arm_request_failed_ = true;
            arm_failure_message_ = "balance state missing or stale after arm waypoint";
            return;
          }
          RCLCPP_INFO(
            get_logger(), "Phase %d alpha %.3f: stability margin %.3f mm, safe=%s.",
            active_arm_phase_, current_arm_waypoint().alpha,
            stability->signed_margin * 1000.0,
            stability->inside_safe_region ? "true" : "false");
          if (stability->inside_safe_region) {
            safe_hold_pending_ = true;
            safe_hold_start_time_ = get_clock()->now();
          } else {
            advance_closed_loop_waypoint();
          }
          return;
        }

        if (active_arm_waypoint_ + 1 < arm_waypoints_.size()) {
          ++active_arm_waypoint_;
          pending_arm_target_ = current_arm_waypoint().positions;
          arm_goal_sent_ = false;
          arm_goal_wait_start_time_ = get_clock()->now();
          publish_stair_phase(current_arm_waypoint().name, true);
          RCLCPP_INFO(
            get_logger(), "Advancing Kinova phase %d to waypoint %zu/%zu (%s).",
            active_arm_phase_, active_arm_waypoint_ + 1, arm_waypoints_.size(),
            current_arm_waypoint().name.c_str());
        } else {
          arm_reached_ = true;
        }
      };

    try {
      arm_goal_sent_ = true;
      arm_client_->async_send_goal(goal, options);
      RCLCPP_INFO(
        get_logger(), "Sent Kinova phase %d waypoint %zu/%zu (%s) via JointPtp (%.2f s).",
        active_arm_phase_, active_arm_waypoint_ + 1, arm_waypoints_.size(),
        current_arm_waypoint().name.c_str(), arm_ptp_duration_sec_);
    } catch (const std::exception & error) {
      arm_goal_sent_ = false;
      arm_request_failed_ = true;
      arm_failure_message_ = error.what();
    }
  }

  void cancel_active_arm_goal()
  {
    if (active_arm_goal_ && arm_client_) {
      arm_client_->async_cancel_goal(active_arm_goal_);
      active_arm_goal_.reset();
    }
  }

  struct ArmWaypoint
  {
    std::string name;
    std::vector<double> positions;
    double alpha{-1.0};
  };

  const ArmWaypoint & current_arm_waypoint() const
  {
    return arm_waypoints_.at(active_arm_waypoint_);
  }

  static const char * arm_phase_name(int phase)
  {
    switch (phase) {
      case 1:
        return "front-left";
      case 2:
        return "front-right";
      case 3:
        return "rear-left";
      case 4:
        return "rear-right";
      default:
        return "unknown";
    }
  }

  void fill_header(kilin_msgs::msg::MotorCmdStamped & msg)
  {
    msg.header.seq = sequence_++;
    const auto now = get_clock()->now();
    msg.header.time.sec = static_cast<int32_t>(now.seconds());
    msg.header.time.nanosec = now.nanoseconds() % 1000000000;
    msg.header.frame_id = "kilin_stair_controller";
  }

  void publish_trigger(bool enable)
  {
    kilin_msgs::msg::TriggerStamped msg;
    msg.enable = enable;
    trigger_pub_->publish(msg);
  }

  void publish_stair_phase(const std::string & waypoint, bool gait_paused)
  {
    kilin_msgs::msg::StairPhaseStamped msg;
    msg.header.seq = phase_sequence_++;
    const auto now = get_clock()->now();
    msg.header.time.sec = static_cast<int32_t>(now.seconds());
    msg.header.time.nanosec = now.nanoseconds() % 1000000000;
    msg.header.frame_id = "world";
    msg.phase = active_arm_phase_;
    msg.waypoint = waypoint;
    msg.gait_paused = gait_paused;
    phase_pub_->publish(msg);
  }

  bool init_trigger_gpio()
  {
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    if (trigger_initialized_) {
      return true;
    }
    trigger_chip_ = gpiod_chip_open(trigger_chipname_.c_str());
    if (!trigger_chip_) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s", trigger_chipname_.c_str());
      return false;
    }
    trigger_line_ = gpiod_chip_get_line(trigger_chip_, trigger_line_offset_);
    if (!trigger_line_ || gpiod_line_request_output(trigger_line_, "kilin_stair_trigger", 1) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to request GPIO line %u", trigger_line_offset_);
      cleanup_trigger_gpio_unlocked();
      return false;
    }
    trigger_initialized_ = true;
    return true;
  }

  void trigger_on()
  {
    if (!init_trigger_gpio()) {
      return;
    }
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    gpiod_line_set_value(trigger_line_, 0);
    trigger_is_on_ = true;
  }

  void trigger_off()
  {
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    if (trigger_initialized_ && trigger_line_) {
      gpiod_line_set_value(trigger_line_, 1);
    }
    trigger_is_on_ = false;
  }

  void cleanup_trigger_gpio()
  {
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    cleanup_trigger_gpio_unlocked();
  }

  void cleanup_trigger_gpio_unlocked()
  {
    if (trigger_line_) {
      gpiod_line_set_value(trigger_line_, 1);
      gpiod_line_release(trigger_line_);
      trigger_line_ = nullptr;
    }
    if (trigger_chip_) {
      gpiod_chip_close(trigger_chip_);
      trigger_chip_ = nullptr;
    }
    trigger_initialized_ = false;
    trigger_is_on_ = false;
  }

  void finish(const char * reason)
  {
    if (finished_) {
      return;
    }
    finished_ = true;
    cancel_active_arm_goal();
    trigger_off();
    publish_trigger(false);
    RCLCPP_INFO(get_logger(), "Stair controller stopped: %s", reason);
    rclcpp::shutdown();
  }

  std::vector<GaitPoint> gait_;
  std::string csv_path_;
  double rate_hz_{};
  double delay_start_sec_{};
  bool arm_enabled_{};
  double arm_timeout_sec_{};
  double arm_ptp_duration_sec_{};
  std::string arm_action_name_;
  std::string arm_control_mode_;
  std::string balance_state_topic_;
  std::string stability_state_topic_;
  bool closed_loop_arm_{false};
  double balance_state_timeout_sec_{};
  double com_safe_margin_m_{};
  double com_alpha_step_{};
  double com_safe_hold_sec_{};
  double amr_yaw_in_world_rad_{};
  std::vector<double> standard_pose_;
  std::vector<double> full_extension_pose_;
  std::vector<double> held_arm_pose_;
  std::array<std::vector<double>, 4> arm_poses_;
  std::deque<ArmWaypoint> arm_waypoints_;
  std::size_t active_arm_waypoint_{0};
  std::vector<double> pending_arm_target_;
  bool csv_has_arm_phase_{false};
  bool invert_ab_hips_{false};

  rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr motor_pub_;
  rclcpp::Publisher<kilin_msgs::msg::TriggerStamped>::SharedPtr trigger_pub_;
  rclcpp::Publisher<kilin_msgs::msg::StairPhaseStamped>::SharedPtr phase_pub_;
  rclcpp::Publisher<kilin_msgs::msg::StabilityStateStamped>::SharedPtr stability_pub_;
  rclcpp::Subscription<kilin_msgs::msg::BalanceStateStamped>::SharedPtr balance_sub_;
  rclcpp_action::Client<JointPtp>::SharedPtr arm_client_;
  JointPtpGoalHandle::SharedPtr active_arm_goal_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  rclcpp::Time playback_start_time_;
  rclcpp::Time arm_wait_start_time_;
  rclcpp::Time arm_goal_wait_start_time_;
  rclcpp::Time last_balance_receive_time_;
  rclcpp::Time safe_hold_start_time_;
  bool started_{false};
  bool waiting_for_arm_{false};
  bool arm_reached_{false};
  bool arm_goal_sent_{false};
  bool arm_request_failed_{false};
  bool safe_hold_pending_{false};
  bool closed_loop_standard_pending_{false};
  bool closed_loop_waiting_for_standard_{false};
  bool have_balance_state_{false};
  std::string arm_failure_message_;
  bool finished_{false};
  // -1 forces the initial CSV phase (including phase 0) to be commanded.
  int active_arm_phase_{-1};
  int previous_arm_phase_{-1};
  uint32_t sequence_{0};
  uint32_t phase_sequence_{0};
  kilin_msgs::msg::MotorCmdStamped last_motor_command_;
  bool last_motor_command_valid_{false};
  kilin_msgs::msg::BalanceStateStamped latest_balance_state_;

  std::string trigger_chipname_;
  unsigned int trigger_line_offset_{112};
  gpiod_chip * trigger_chip_{nullptr};
  gpiod_line * trigger_line_{nullptr};
  std::mutex trigger_mutex_;
  bool trigger_initialized_{false};
  bool trigger_is_on_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigint_handler);
  try {
    rclcpp::spin(std::make_shared<KilinStairController>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("kilin_stair_controller"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
