// Copyright 2026 Ian

#pragma once

#include <Eigen/Geometry>

#include <array>
#include <optional>

namespace kilin_com_estimator
{

struct TerrainOrientation
{
  Eigen::Vector3d vertical_in_base;
  Eigen::Matrix3d base_to_level;
};

std::optional<TerrainOrientation> estimate_terrain_orientation(
  const std::array<Eigen::Vector3d, 4> & contact_points,
  const std::array<int, 4> & tread_levels,
  const std::array<bool, 4> & supported,
  double stair_rise_m,
  const std::optional<Eigen::Vector3d> & previous_vertical = std::nullopt);

}  // namespace kilin_com_estimator
