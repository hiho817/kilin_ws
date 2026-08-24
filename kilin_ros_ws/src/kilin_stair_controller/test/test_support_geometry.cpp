// Copyright 2026 Ian

#include <array>
#include <cmath>

#include "kilin_stair_controller/support_geometry.hpp"
#include "gtest/gtest.h"

namespace geometry = kilin_stair_controller::geometry;

TEST(SupportGeometry, ExcludesTheSwingWheel)
{
  const std::array<geometry::Point2, 4> wheels = {{
    {1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}}};
  const auto result = geometry::evaluate_stability({0.0, 0.0}, wheels, 2, 0.1);
  ASSERT_EQ(result.hull.size(), 3U);
  EXPECT_NEAR(result.signed_margin, 0.0, 1e-12);
  EXPECT_FALSE(result.inside_safe_region);
  EXPECT_GT(result.correction_distance, 0.0);
}

TEST(SupportGeometry, FindsTrueClosestPointToInsetTriangle)
{
  const std::array<geometry::Point2, 4> wheels = {{
    {1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}}};
  const auto result = geometry::evaluate_stability({-0.9, -0.9}, wheels, 4, 0.1);
  EXPECT_FALSE(result.inside_safe_region);
  EXPECT_GT(result.direction.x, 0.0);
  EXPECT_GT(result.direction.y, 0.0);
  EXPECT_NEAR(std::hypot(result.direction.x, result.direction.y), 1.0, 1e-12);
}

TEST(SupportGeometry, ReportsSafePointInsideInsetTriangle)
{
  const std::array<geometry::Point2, 4> wheels = {{
    {1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}}};
  const auto result = geometry::evaluate_stability({0.5, 0.5}, wheels, 4, 0.1);
  EXPECT_TRUE(result.inside_support);
  EXPECT_TRUE(result.inside_safe_region);
  EXPECT_DOUBLE_EQ(result.correction_distance, 0.0);
}

TEST(ArmGeometry, AppliesBaseYawOffsetToCardinalDirections)
{
  constexpr double pi = 3.14159265358979323846;

  EXPECT_NEAR(geometry::arm_joint1_for_direction({1.0, 0.0}, 0.0), 0.0, 1e-12);
  EXPECT_NEAR(geometry::arm_joint1_for_direction({0.0, 1.0}, 0.0), -pi / 2.0, 1e-12);
  EXPECT_NEAR(geometry::arm_joint1_for_direction({1.0, 0.0}, pi), pi, 1e-12);
  EXPECT_NEAR(geometry::arm_joint1_for_direction({0.0, 1.0}, pi), pi / 2.0, 1e-12);
  EXPECT_NEAR(geometry::arm_joint1_for_direction({-1.0, 0.0}, pi), 0.0, 1e-12);
}

TEST(ArmGeometry, RejectsZeroDirection)
{
  EXPECT_THROW(geometry::arm_joint1_for_direction({0.0, 0.0}, 0.0), std::runtime_error);
}
