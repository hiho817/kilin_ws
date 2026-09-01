// Copyright 2026 Ian

#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "kilin_stair_controller/com_inverse.hpp"

namespace inverse = kilin_stair_controller::inverse;
namespace geometry = kilin_stair_controller::geometry;

TEST(ComInverse, HorizontalDistanceModelIsMonotonic)
{
  double previous = inverse::fitted_dxy(0.0);
  for (int index = 1; index <= 1000; ++index) {
    const double current = inverse::fitted_dxy(static_cast<double>(index) / 1000.0);
    EXPECT_GT(current, previous);
    previous = current;
  }
  EXPECT_NE(inverse::fitted_dxy(1.0), 0.0);
}

TEST(ComInverse, FeedforwardTargetTracksPhaseReleaseMargin)
{
  EXPECT_NEAR(inverse::feedforward_target_margin(0.025, 0.001), 0.024, 1e-12);
  EXPECT_NEAR(inverse::feedforward_target_margin(0.023, 0.001), 0.022, 1e-12);
  EXPECT_NEAR(inverse::feedforward_target_margin(0.015, 0.001), 0.014, 1e-12);
  EXPECT_DOUBLE_EQ(inverse::feedforward_target_margin(0.0005, 0.001), 0.0);
}

TEST(ComInverse, FeedforwardTargetRejectsInvalidMargins)
{
  EXPECT_THROW(inverse::feedforward_target_margin(-0.001, 0.001), std::runtime_error);
  EXPECT_THROW(inverse::feedforward_target_margin(0.015, -0.001), std::runtime_error);
}

TEST(ComInverse, FindsMinimumAlphaInFlatOutputFrame)
{
  const double required = inverse::fitted_dxy(0.5);
  const std::vector<geometry::Point2> polygon =
  {{required, -0.02}, {0.1, -0.02}, {0.1, 0.02}, {required, 0.02}};
  const auto result = inverse::solve_minimum_alpha(
    {0.0, 0.0}, polygon, {1.0, 0.0}, inverse::identity_rotation());
  ASSERT_TRUE(result.reachable);
  EXPECT_NEAR(result.alpha, 0.5, 1e-6);
  EXPECT_GE(result.predicted_safe_margin, -1e-9);
}

TEST(ComInverse, IncludesVerticalDeltaComAtNonzeroPitch)
{
  constexpr double pi = 3.14159265358979323846;
  const double pitch = 20.0 * pi / 180.0;
  const inverse::Matrix3 rotation = {{{
    std::cos(pitch), 0.0, std::sin(pitch)},
    {0.0, 1.0, 0.0},
    {-std::sin(pitch), 0.0, std::cos(pitch)}}};
  const double expected_alpha = 0.55;
  const auto expected_delta = inverse::predicted_world_delta(
    expected_alpha, {1.0, 0.0}, rotation);
  const std::vector<geometry::Point2> polygon = {
    {expected_delta.x, -0.02}, {0.2, -0.02}, {0.2, 0.02}, {expected_delta.x, 0.02}};
  const auto result = inverse::solve_minimum_alpha(
    {0.0, 0.0}, polygon, {1.0, 0.0}, rotation);
  ASSERT_TRUE(result.reachable);
  EXPECT_NEAR(result.alpha, expected_alpha, 1e-6);
}

TEST(ComInverse, ReplaysPostJoint1IsaacCase)
{
  const std::array<geometry::Point2, 4> wheels = {{
    {-0.055646583, 0.249164835},
    {-0.055641450, -0.246835619},
    {-0.535658002, 0.249155343},
    {-0.535651445, -0.246845186}}};
  const geometry::Point2 post_joint1_com{-0.253789461, 0.020181360};
  const auto stability = geometry::evaluate_stability(post_joint1_com, wheels, 1, 0.01);
  constexpr double pi = 3.14159265358979323846;
  const double joint1 = 135.960789392 * pi / 180.0;
  const geometry::Point2 extension_direction{std::cos(joint1), -std::sin(joint1)};
  const auto result = inverse::solve_minimum_alpha(
    post_joint1_com, stability.safe_polygon, extension_direction,
    inverse::identity_rotation());
  ASSERT_TRUE(result.reachable);
  EXPECT_NEAR(result.alpha, 0.60455, 5e-5);
}

TEST(ComInverse, ReportsUnreachableDirectionAtFullExtension)
{
  const std::vector<geometry::Point2> polygon =
  {{0.02, -0.02}, {0.04, -0.02}, {0.04, 0.02}, {0.02, 0.02}};
  const auto result = inverse::solve_minimum_alpha(
    {0.0, 0.0}, polygon, {-1.0, 0.0}, inverse::identity_rotation());
  EXPECT_FALSE(result.reachable);
  EXPECT_TRUE(result.saturated);
  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
}
