// Copyright 2026 Ian

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <array>

#include "kilin_com_estimator/terrain_orientation.hpp"

namespace
{

TEST(TerrainOrientation, RecoversKnownPitchFromThreeSupports)
{
  const double pitch = 20.0 * M_PI / 180.0;
  const Eigen::Matrix3d base_to_level =
    Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d vertical_in_base = base_to_level.row(2).transpose();
  const double rise = 0.10;
  const std::array<int, 4> levels = {2, 2, 1, 1};
  const std::array<bool, 4> supported = {true, false, true, true};

  const std::array<Eigen::Vector3d, 4> world_points = {
    Eigen::Vector3d(0.25, 0.25, 2.0 * rise),
    Eigen::Vector3d(0.25, -0.25, 2.0 * rise),
    Eigen::Vector3d(-0.25, 0.25, rise),
    Eigen::Vector3d(-0.25, -0.25, rise)};
  std::array<Eigen::Vector3d, 4> base_points;
  for (std::size_t i = 0; i < base_points.size(); ++i) {
    base_points[i] = base_to_level.transpose() * world_points[i];
  }

  const auto result = kilin_com_estimator::estimate_terrain_orientation(
    base_points, levels, supported, rise);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(0.0, (result->vertical_in_base - vertical_in_base).norm(), 1e-9);
  EXPECT_NEAR(0.0, (result->base_to_level - base_to_level).norm(), 1e-9);
}

TEST(TerrainOrientation, UsesBestConditionedTripletWithFourSupports)
{
  const std::array<Eigen::Vector3d, 4> points = {
    Eigen::Vector3d(0.25, 0.25, 0.0), Eigen::Vector3d(0.25, -0.25, 0.0),
    Eigen::Vector3d(-0.25, 0.25, 0.0), Eigen::Vector3d(-0.25, -0.25, 0.0)};
  const std::array<int, 4> levels = {0, 0, 0, 1};
  const std::array<bool, 4> supported = {true, true, true, true};
  EXPECT_TRUE(
    kilin_com_estimator::estimate_terrain_orientation(
      points, levels, supported, 0.10).has_value());
}

}  // namespace
