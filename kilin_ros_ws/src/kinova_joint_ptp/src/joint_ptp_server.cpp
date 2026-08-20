// Copyright 2026 Ian
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kinova_joint_ptp/joint_ptp_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>
#include <utility>

#include "kinova_joint_ptp/joint_math.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

namespace kinova_joint_ptp
{

JointPtpServer::JointPtpServer(const rclcpp::NodeOptions & options)
: Node("kinova_joint_ptp", options)
{
  action_name_ = declare_parameter<std::string>("action_name", "kinova_joint_ptp");
  controller_action_name_ = declare_parameter<std::string>(
    "controller_action_name", "/joint_trajectory_controller/follow_joint_trajectory");
  joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
  expected_joint_names_ = declare_parameter<std::vector<std::string>>(
    "joint_names",
    {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"});
  const auto continuous_joint_names = declare_parameter<std::vector<std::string>>(
    "continuous_joint_names", {"joint_1", "joint_3", "joint_5", "joint_7"});
  controller_wait_timeout_sec_ =
    declare_parameter<double>("controller_wait_timeout_sec", 3.0);
  goal_response_timeout_sec_ = declare_parameter<double>("goal_response_timeout_sec", 3.0);
  result_timeout_margin_sec_ = declare_parameter<double>("result_timeout_margin_sec", 12.0);
  final_joint_tolerance_rad_ = declare_parameter<double>("final_joint_tolerance_rad", 0.03);
  max_duration_sec_ = declare_parameter<double>("max_duration_sec", 60.0);
  require_all_joints_ = declare_parameter<bool>("require_all_joints", true);
  require_joint_states_ = declare_parameter<bool>("require_joint_states", true);

  if (action_name_.empty() || controller_action_name_.empty() || joint_state_topic_.empty()) {
    throw std::runtime_error("Action and topic names must not be empty");
  }
  if (expected_joint_names_.empty()) {
    throw std::runtime_error("joint_names must not be empty");
  }
  if (controller_wait_timeout_sec_ <= 0.0 || goal_response_timeout_sec_ <= 0.0 ||
    result_timeout_margin_sec_ <= 0.0 || final_joint_tolerance_rad_ <= 0.0 ||
    max_duration_sec_ <= 0.0)
  {
    throw std::runtime_error(
            "All PTP timeout, tolerance, and duration parameters must be positive");
  }

  expected_joint_name_set_.insert(expected_joint_names_.begin(), expected_joint_names_.end());
  if (expected_joint_name_set_.size() != expected_joint_names_.size()) {
    throw std::runtime_error("joint_names contains duplicates");
  }
  continuous_joint_names_.insert(continuous_joint_names.begin(), continuous_joint_names.end());
  for (const auto & name : continuous_joint_names_) {
    if (expected_joint_name_set_.count(name) == 0U) {
      throw std::runtime_error("continuous_joint_names contains an unknown joint: " + name);
    }
  }

  trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, controller_action_name_);
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      const auto count = std::min(msg->name.size(), msg->position.size());
      for (std::size_t index = 0; index < count; ++index) {
        if (std::isfinite(msg->position[index])) {
          joint_positions_[msg->name[index]] = msg->position[index];
        }
      }
    });

  action_server_ = rclcpp_action::create_server<JointPtp>(
    this,
    action_name_,
    std::bind(
      &JointPtpServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&JointPtpServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&JointPtpServer::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Joint PTP server ready: action='%s', controller='%s', joint_states='%s'",
    action_name_.c_str(), controller_action_name_.c_str(), joint_state_topic_.c_str());
}

JointPtpServer::~JointPtpServer()
{
  FollowJointTrajectoryGoalHandle::SharedPtr trajectory_goal;
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    trajectory_goal = active_trajectory_goal_;
  }
  if (trajectory_goal && trajectory_client_) {
    trajectory_client_->async_cancel_goal(trajectory_goal);
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

rclcpp_action::GoalResponse JointPtpServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const JointPtp::Goal> goal)
{
  std::string reason;
  if (!validate_goal(*goal, reason)) {
    RCLCPP_WARN(get_logger(), "Rejected PTP goal: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (require_joint_states_ && !has_required_joint_states(goal->joint_names)) {
    RCLCPP_WARN(get_logger(), "Rejected PTP goal: required joint states are not available");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::lock_guard<std::mutex> lock(active_goal_mutex_);
  if (goal_active_) {
    RCLCPP_WARN(get_logger(), "Rejected PTP goal: another goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  goal_active_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointPtpServer::handle_cancel(
  const std::shared_ptr<JointPtpGoalHandle>)
{
  FollowJointTrajectoryGoalHandle::SharedPtr trajectory_goal;
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    trajectory_goal = active_trajectory_goal_;
  }
  if (trajectory_goal) {
    trajectory_client_->async_cancel_goal(trajectory_goal);
  }
  RCLCPP_INFO(get_logger(), "PTP cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointPtpServer::handle_accepted(const std::shared_ptr<JointPtpGoalHandle> goal_handle)
{
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  worker_thread_ = std::thread(&JointPtpServer::execute, this, goal_handle);
}

void JointPtpServer::execute(const std::shared_ptr<JointPtpGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<JointPtp::Result>();
  result->success = false;
  result->error_code = JointPtp::Result::INTERNAL_ERROR;
  result->final_max_joint_error = -1.0;

  const auto finish_abort =
    [this, &goal_handle, &result](int32_t error_code, const std::string & message) {
      result->success = false;
      result->error_code = error_code;
      result->message = message;
      if (goal_handle->is_active()) {
        goal_handle->abort(result);
      }
      RCLCPP_ERROR(get_logger(), "PTP aborted: %s", message.c_str());
      clear_active_goal();
    };
  const auto finish_canceled =
    [this, &goal_handle, &result](const std::string & message) {
      result->success = false;
      result->error_code = JointPtp::Result::CANCELED;
      result->message = message;
      if (goal_handle->is_active()) {
        goal_handle->canceled(result);
      }
      RCLCPP_WARN(get_logger(), "PTP canceled: %s", message.c_str());
      clear_active_goal();
    };

  try {
    if (!trajectory_client_->wait_for_action_server(
        std::chrono::duration<double>(controller_wait_timeout_sec_)))
    {
      finish_abort(
        JointPtp::Result::CONTROLLER_UNAVAILABLE,
        "FollowJointTrajectory controller is not available: " + controller_action_name_);
      return;
    }

    const auto targets = unwrap_targets(goal->joint_names, goal->positions);
    FollowJointTrajectory::Goal trajectory_goal;
    trajectory_goal.trajectory.joint_names = goal->joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = targets;
    if (!goal->velocities.empty()) {
      point.velocities = goal->velocities;
    }
    const auto duration_nanoseconds = static_cast<int64_t>(
      std::llround(goal->duration_sec * 1e9));
    point.time_from_start.sec = static_cast<int32_t>(duration_nanoseconds / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(
      duration_nanoseconds % 1000000000LL);
    trajectory_goal.trajectory.points.push_back(std::move(point));

    const auto start = std::chrono::steady_clock::now();
    auto send_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_options.feedback_callback =
      [this, goal_handle, names = goal->joint_names, targets, duration = goal->duration_sec, start](
      FollowJointTrajectoryGoalHandle::SharedPtr,
      const std::shared_ptr<const FollowJointTrajectory::Feedback>) {
        publish_feedback(goal_handle, names, targets, duration, start, "executing");
      };

    auto trajectory_goal_future = trajectory_client_->async_send_goal(
      trajectory_goal, send_options);
    const auto goal_response_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(goal_response_timeout_sec_));
    while (rclcpp::ok() &&
      trajectory_goal_future.wait_for(50ms) != std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() >= goal_response_deadline) {
        finish_abort(JointPtp::Result::TIMEOUT, "Timed out waiting for controller goal response");
        return;
      }
    }
    if (!rclcpp::ok()) {
      clear_active_goal();
      return;
    }

    const auto accepted_trajectory_goal = trajectory_goal_future.get();
    if (!accepted_trajectory_goal) {
      finish_abort(JointPtp::Result::CONTROLLER_REJECTED, "Controller rejected trajectory goal");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_trajectory_goal_ = accepted_trajectory_goal;
    }

    if (goal_handle->is_canceling()) {
      trajectory_client_->async_cancel_goal(accepted_trajectory_goal);
    }

    auto trajectory_result_future = trajectory_client_->async_get_result(accepted_trajectory_goal);
    const auto result_deadline = start +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(goal->duration_sec + result_timeout_margin_sec_));
    while (rclcpp::ok() &&
      trajectory_result_future.wait_for(50ms) != std::future_status::ready)
    {
      if (goal_handle->is_canceling()) {
        trajectory_client_->async_cancel_goal(accepted_trajectory_goal);
      }
      publish_feedback(
        goal_handle, goal->joint_names, targets, goal->duration_sec, start,
        goal_handle->is_canceling() ? "canceling" : "executing");
      if (std::chrono::steady_clock::now() >= result_deadline) {
        trajectory_client_->async_cancel_goal(accepted_trajectory_goal);
        finish_abort(JointPtp::Result::TIMEOUT, "Trajectory result timed out");
        return;
      }
    }
    if (!rclcpp::ok()) {
      clear_active_goal();
      return;
    }

    const auto trajectory_result = trajectory_result_future.get();
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_trajectory_goal_.reset();
    }

    result->final_max_joint_error = calculate_max_joint_error(goal->joint_names, targets);

    if (goal_handle->is_canceling() ||
      trajectory_result.code == rclcpp_action::ResultCode::CANCELED)
    {
      finish_canceled("Trajectory was canceled");
      return;
    }

    if (trajectory_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      finish_abort(JointPtp::Result::TRAJECTORY_ABORTED, "Controller aborted trajectory");
      return;
    }
    if (!trajectory_result.result ||
      trajectory_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      const auto controller_message = trajectory_result.result ?
        trajectory_result.result->error_string : "Controller returned no result payload";
      finish_abort(
        JointPtp::Result::TRAJECTORY_ABORTED,
        "Controller reported trajectory failure: " + controller_message);
      return;
    }
    if (result->final_max_joint_error < 0.0) {
      finish_abort(
        JointPtp::Result::FINAL_ERROR_TOO_LARGE,
        "Final joint error is unavailable because joint states are missing");
      return;
    }
    if (result->final_max_joint_error > final_joint_tolerance_rad_) {
      finish_abort(
        JointPtp::Result::FINAL_ERROR_TOO_LARGE,
        "Final joint error exceeds tolerance");
      return;
    }

    result->success = true;
    result->error_code = JointPtp::Result::SUCCESS;
    result->message = "Joint PTP trajectory succeeded";
    goal_handle->succeed(result);
    RCLCPP_INFO(
      get_logger(), "PTP succeeded; final max joint error=%.6f rad",
      result->final_max_joint_error);
    clear_active_goal();
  } catch (const std::exception & error) {
    finish_abort(JointPtp::Result::INTERNAL_ERROR, error.what());
  }
}

bool JointPtpServer::validate_goal(const JointPtp::Goal & goal, std::string & reason) const
{
  if (goal.joint_names.empty()) {
    reason = "joint_names must not be empty";
    return false;
  }
  if (goal.joint_names.size() != goal.positions.size()) {
    reason = "joint_names and positions sizes do not match";
    return false;
  }
  if (!goal.velocities.empty() && goal.velocities.size() != goal.positions.size()) {
    reason = "velocities must be empty or match positions size";
    return false;
  }
  if (!std::isfinite(goal.duration_sec) || goal.duration_sec <= 0.0 ||
    goal.duration_sec > max_duration_sec_)
  {
    reason = "duration_sec must be finite, positive, and no greater than max_duration_sec";
    return false;
  }

  std::unordered_set<std::string> names;
  for (std::size_t index = 0; index < goal.joint_names.size(); ++index) {
    const auto & name = goal.joint_names[index];
    if (name.empty()) {
      reason = "joint name must not be empty";
      return false;
    }
    if (!names.insert(name).second) {
      reason = "joint_names contains duplicate joint: " + name;
      return false;
    }
    if (expected_joint_name_set_.count(name) == 0U) {
      reason = "goal contains a joint outside the configured whitelist: " + name;
      return false;
    }
    if (!std::isfinite(goal.positions[index])) {
      reason = "positions contains a non-finite value";
      return false;
    }
    if (!goal.velocities.empty() && !std::isfinite(goal.velocities[index])) {
      reason = "velocities contains a non-finite value";
      return false;
    }
  }
  if (require_all_joints_ && names != expected_joint_name_set_) {
    reason = "goal must contain every configured Kinova joint exactly once";
    return false;
  }
  return true;
}

bool JointPtpServer::has_required_joint_states(
  const std::vector<std::string> & joint_names) const
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  return std::all_of(
    joint_names.begin(), joint_names.end(),
    [this](const std::string & name) {return joint_positions_.count(name) != 0U;});
}

std::vector<double> JointPtpServer::unwrap_targets(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions) const
{
  auto targets = positions;
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    const auto & name = joint_names[index];
    if (continuous_joint_names_.count(name) == 0U) {
      continue;
    }
    const auto current = joint_positions_.find(name);
    if (current == joint_positions_.end()) {
      if (require_joint_states_) {
        throw std::runtime_error("Missing joint state for continuous joint: " + name);
      }
      RCLCPP_WARN(get_logger(), "Missing state for %s; shortest-path unwrap skipped", name.c_str());
      continue;
    }
    const auto original = targets[index];
    targets[index] = shortest_equivalent_target(current->second, original);
    RCLCPP_DEBUG(
      get_logger(), "Unwrapped %s target %.6f -> %.6f from current %.6f",
      name.c_str(), original, targets[index], current->second);
  }
  return targets;
}

double JointPtpServer::calculate_max_joint_error(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & targets) const
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  double max_error = 0.0;
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    const auto current = joint_positions_.find(joint_names[index]);
    if (current == joint_positions_.end()) {
      return -1.0;
    }
    auto error = targets[index] - current->second;
    if (continuous_joint_names_.count(joint_names[index]) != 0U) {
      error = shortest_angular_error(current->second, targets[index]);
    }
    max_error = std::max(max_error, std::abs(error));
  }
  return max_error;
}

void JointPtpServer::publish_feedback(
  const std::shared_ptr<JointPtpGoalHandle> & goal_handle,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & targets,
  double duration_sec,
  const std::chrono::steady_clock::time_point & start,
  const std::string & state) const
{
  if (!goal_handle->is_active()) {
    return;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();
  auto feedback = std::make_shared<JointPtp::Feedback>();
  feedback->elapsed_sec = elapsed;
  feedback->remaining_sec = std::max(0.0, duration_sec - elapsed);
  feedback->progress = std::clamp(elapsed / duration_sec, 0.0, 1.0);
  feedback->max_joint_error = calculate_max_joint_error(joint_names, targets);
  feedback->state = state;
  goal_handle->publish_feedback(feedback);
}

void JointPtpServer::clear_active_goal()
{
  std::lock_guard<std::mutex> lock(active_goal_mutex_);
  active_trajectory_goal_.reset();
  goal_active_ = false;
}

}  // namespace kinova_joint_ptp
