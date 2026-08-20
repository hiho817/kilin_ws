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

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "kinova_joint_ptp/joint_ptp_server.hpp"
#include "kinova_ptp_interfaces/action/joint_ptp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace kinova_joint_ptp
{

using JointPtpAction = kinova_ptp_interfaces::action::JointPtp;

class JointPtpServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

class ExecutorThreadGuard
{
public:
  explicit ExecutorThreadGuard(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorThreadGuard()
  {
    executor_.cancel();
    thread_.join();
  }

private:
  rclcpp::Executor & executor_;
  std::thread thread_;
};

TEST_F(JointPtpServerTest, RejectsInvalidGoalAndReportsMissingController)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("action_name", "/test_kinova_joint_ptp"),
      rclcpp::Parameter(
        "controller_action_name", "/controller_that_does_not_exist/follow_joint_trajectory"),
      rclcpp::Parameter("joint_names", std::vector<std::string>{"joint_1"}),
      rclcpp::Parameter("continuous_joint_names", std::vector<std::string>{"joint_1"}),
      rclcpp::Parameter("controller_wait_timeout_sec", 0.1),
      rclcpp::Parameter("require_joint_states", false)});

  auto server = std::make_shared<JointPtpServer>(options);
  auto client_node = std::make_shared<rclcpp::Node>("joint_ptp_test_client");
  auto client = rclcpp_action::create_client<JointPtpAction>(
    client_node, "/test_kinova_joint_ptp");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(server);
  executor.add_node(client_node);
  ExecutorThreadGuard executor_thread(executor);

  ASSERT_TRUE(client->wait_for_action_server(2s));

  JointPtpAction::Goal invalid_goal;
  invalid_goal.joint_names = {"joint_1"};
  invalid_goal.positions = {0.0};
  invalid_goal.duration_sec = 0.0;
  auto invalid_goal_future = client->async_send_goal(invalid_goal);
  ASSERT_EQ(invalid_goal_future.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(invalid_goal_future.get(), nullptr);

  JointPtpAction::Goal valid_goal;
  valid_goal.joint_names = {"joint_1"};
  valid_goal.positions = {0.0};
  valid_goal.duration_sec = 1.0;
  auto valid_goal_future = client->async_send_goal(valid_goal);
  ASSERT_EQ(valid_goal_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = valid_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_FALSE(result.result->success);
  EXPECT_EQ(result.result->error_code, JointPtpAction::Result::CONTROLLER_UNAVAILABLE);
}

}  // namespace kinova_joint_ptp
