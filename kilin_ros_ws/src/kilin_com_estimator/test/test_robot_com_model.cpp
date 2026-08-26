// Copyright 2026 Ian

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "kilin_com_estimator/com_bias.hpp"
#include "kilin_com_estimator/joint_mapping.hpp"
#include "kilin_com_estimator/robot_com_model.hpp"

namespace
{
using kilin_com_estimator::RobotComModel;

TEST(ComBias, ParsesThreeFiniteBaseFrameValues)
{
  const auto bias = kilin_com_estimator::parse_com_bias({0.0054, -0.0011, 0.0});
  EXPECT_DOUBLE_EQ(bias.x(), 0.0054);
  EXPECT_DOUBLE_EQ(bias.y(), -0.0011);
  EXPECT_DOUBLE_EQ(bias.z(), 0.0);
}

TEST(ComBias, RejectsWrongSizeAndNonFiniteValues)
{
  EXPECT_THROW(kilin_com_estimator::parse_com_bias({0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(
    kilin_com_estimator::parse_com_bias(
      {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0}),
    std::invalid_argument);
}

TEST(JointMapping, UsesNamesAndIgnoresRobotiqJoint)
{
  const std::vector<std::string> names = {
    "joint_1", "robotiq_85_left_knuckle_joint", "joint_2", "joint_4",
    "joint_5", "joint_3", "joint_6", "joint_7"};
  const std::vector<double> positions = {1.0, 99.0, 2.0, 4.0, 5.0, 3.0, 6.0, 7.0};
  const std::vector<std::string> input = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
  const std::vector<std::string> output = {
    "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4",
    "arm_joint_5", "arm_joint_6", "arm_joint_7"};
  std::map<std::string, double> mapped;
  std::string error;

  ASSERT_TRUE(
    kilin_com_estimator::map_named_arm_positions(
      names, positions, input, output, mapped, error)) << error;
  ASSERT_EQ(mapped.size(), 7U);
  for (int index = 1; index <= 7; ++index) {
    EXPECT_DOUBLE_EQ(mapped.at("arm_joint_" + std::to_string(index)), index);
  }
}

TEST(JointMapping, RejectsMissingRequiredJoint)
{
  std::map<std::string, double> mapped;
  std::string error;
  EXPECT_FALSE(
    kilin_com_estimator::map_named_arm_positions(
      {"joint_1"}, {1.0}, {"joint_1", "joint_2"},
      {"arm_joint_1", "arm_joint_2"}, mapped, error));
  EXPECT_NE(error.find("joint_2"), std::string::npos);
  EXPECT_TRUE(mapped.empty());
}

TEST(JointMapping, PositionDiffIsActualMinusPosition)
{
  EXPECT_DOUBLE_EQ(kilin_com_estimator::actual_motor_position(0.4, -0.05), 0.35);
  EXPECT_DOUBLE_EQ(kilin_com_estimator::actual_motor_position(-0.2, 0.03), -0.17);
}

TEST(RobotComModel, LoadsExpectedMassAndFiniteGeometry)
{
  const RobotComModel model(TEST_URDF_PATH);
  const auto result = model.compute({});
  EXPECT_NEAR(model.model_mass(), 41.8929641, 1e-7);
  EXPECT_NEAR(result.total_mass, model.model_mass(), 1e-12);
  EXPECT_TRUE(result.com.allFinite());
  for (const auto & wheel : result.wheel_centers) {
    EXPECT_TRUE(wheel.allFinite());
  }
}

TEST(RobotComModel, FrontLeftHipOnlyMovesFrontLeftWheel)
{
  const RobotComModel model(TEST_URDF_PATH);
  const auto zero = model.compute({});
  const auto moved = model.compute({{"FL_hip", 0.2}});

  EXPECT_GT((moved.wheel_centers[0] - zero.wheel_centers[0]).norm(), 1e-3);
  for (std::size_t index = 1; index < moved.wheel_centers.size(); ++index) {
    EXPECT_NEAR((moved.wheel_centers[index] - zero.wheel_centers[index]).norm(), 0.0, 1e-12);
  }
}

TEST(RobotComModel, ArmMotionChangesCombinedCom)
{
  const RobotComModel model(TEST_URDF_PATH);
  const auto zero = model.compute({});
  const auto moved = model.compute({{"arm_joint_2", 1.0}, {"arm_joint_4", -0.7}});
  EXPECT_GT((moved.com - zero.com).norm(), 1e-3);
  EXPECT_NEAR(moved.total_mass, zero.total_mass, 1e-12);
}

}  // namespace
