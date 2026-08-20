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

#ifndef KINOVA_JOINT_PTP__JOINT_PTP_SERVER_HPP_
#define KINOVA_JOINT_PTP__JOINT_PTP_SERVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "kinova_ptp_interfaces/action/joint_ptp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace kinova_joint_ptp
{

class JointPtpServer : public rclcpp::Node
{
public:
  using JointPtp = kinova_ptp_interfaces::action::JointPtp;
  using JointPtpGoalHandle = rclcpp_action::ServerGoalHandle<JointPtp>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using FollowJointTrajectoryGoalHandle =
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit JointPtpServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~JointPtpServer() override;

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const JointPtp::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<JointPtpGoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<JointPtpGoalHandle> goal_handle);
  void execute(const std::shared_ptr<JointPtpGoalHandle> goal_handle);

  bool validate_goal(const JointPtp::Goal & goal, std::string & reason) const;
  bool has_required_joint_states(const std::vector<std::string> & joint_names) const;
  std::vector<double> unwrap_targets(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions) const;
  double calculate_max_joint_error(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & targets) const;
  void publish_feedback(
    const std::shared_ptr<JointPtpGoalHandle> & goal_handle,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & targets,
    double duration_sec,
    const std::chrono::steady_clock::time_point & start,
    const std::string & state) const;
  void clear_active_goal();

  std::string action_name_;
  std::string controller_action_name_;
  std::string joint_state_topic_;
  std::vector<std::string> expected_joint_names_;
  std::unordered_set<std::string> expected_joint_name_set_;
  std::unordered_set<std::string> continuous_joint_names_;
  double controller_wait_timeout_sec_;
  double goal_response_timeout_sec_;
  double result_timeout_margin_sec_;
  double final_joint_tolerance_rad_;
  double max_duration_sec_;
  bool require_all_joints_;
  bool require_joint_states_;

  rclcpp_action::Server<JointPtp>::SharedPtr action_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  mutable std::mutex joint_state_mutex_;
  std::unordered_map<std::string, double> joint_positions_;

  mutable std::mutex active_goal_mutex_;
  bool goal_active_{false};
  FollowJointTrajectoryGoalHandle::SharedPtr active_trajectory_goal_;
  std::thread worker_thread_;
};

}  // namespace kinova_joint_ptp

#endif  // KINOVA_JOINT_PTP__JOINT_PTP_SERVER_HPP_
