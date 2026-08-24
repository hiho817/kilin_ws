// Copyright 2026 Ian

#include "kilin_com_estimator/terrain_orientation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace kilin_com_estimator
{

std::optional<TerrainOrientation> estimate_terrain_orientation(
  const std::array<Eigen::Vector3d, 4> & contact_points,
  const std::array<int, 4> & tread_levels,
  const std::array<bool, 4> & supported,
  double stair_rise_m,
  const std::optional<Eigen::Vector3d> & previous_vertical)
{
  if (!std::isfinite(stair_rise_m) || stair_rise_m <= 0.0) {
    return std::nullopt;
  }
  std::vector<std::size_t> indices;
  for (std::size_t i = 0; i < supported.size(); ++i) {
    if (supported[i] && contact_points[i].allFinite()) {
      indices.push_back(i);
    }
  }
  if (indices.size() < 3) {
    return std::nullopt;
  }

  // Select the best-conditioned support triplet.  For three points, the two
  // height-difference equations define an affine line in n; |n|=1 gives two
  // candidates, and positive/upward continuity selects the physical one.
  double best_area = 0.0;
  std::array<std::size_t, 3> best{};
  for (std::size_t a = 0; a + 2 < indices.size(); ++a) {
    for (std::size_t b = a + 1; b + 1 < indices.size(); ++b) {
      for (std::size_t c = b + 1; c < indices.size(); ++c) {
        const Eigen::Vector3d d1 = contact_points[indices[b]] - contact_points[indices[a]];
        const Eigen::Vector3d d2 = contact_points[indices[c]] - contact_points[indices[a]];
        const double area = d1.cross(d2).norm();
        if (area > best_area) {
          best_area = area;
          best = {indices[a], indices[b], indices[c]};
        }
      }
    }
  }
  if (best_area < 1e-8) {
    return std::nullopt;
  }

  Eigen::Matrix<double, 2, 3> a;
  Eigen::Vector2d b;
  for (int row = 0; row < 2; ++row) {
    const std::size_t index = best[static_cast<std::size_t>(row + 1)];
    a.row(row) = (contact_points[index] - contact_points[best[0]]).transpose();
    b[row] = static_cast<double>(tread_levels[index] - tread_levels[best[0]]) * stair_rise_m;
  }
  const Eigen::Matrix2d gram = a * a.transpose();
  if (std::abs(gram.determinant()) < 1e-12) {
    return std::nullopt;
  }
  const Eigen::Vector3d particular = a.transpose() * gram.inverse() * b;
  double remaining = 1.0 - particular.squaredNorm();
  if (remaining < -1e-6) {
    return std::nullopt;
  }
  remaining = std::max(0.0, remaining);
  Eigen::Vector3d null_direction = a.row(0).transpose().cross(a.row(1).transpose());
  null_direction.normalize();
  const double scale = std::sqrt(remaining);
  const std::array<Eigen::Vector3d, 2> candidates = {
    particular + scale * null_direction,
    particular - scale * null_direction};

  Eigen::Vector3d vertical = candidates[0];
  auto score = [&previous_vertical](const Eigen::Vector3d & candidate) {
      if (candidate.z() <= 0.0) {
        return -std::numeric_limits<double>::infinity();
      }
      return previous_vertical ? candidate.dot(previous_vertical->normalized()) : candidate.z();
    };
  if (score(candidates[1]) > score(candidates[0])) {
    vertical = candidates[1];
  }
  if (vertical.z() <= 0.0 || !vertical.allFinite()) {
    return std::nullopt;
  }
  vertical.normalize();

  Eigen::Vector3d level_x = Eigen::Vector3d::UnitX() -
    vertical * vertical.dot(Eigen::Vector3d::UnitX());
  if (level_x.norm() < 1e-6) {
    return std::nullopt;
  }
  level_x.normalize();
  const Eigen::Vector3d level_y = vertical.cross(level_x).normalized();

  TerrainOrientation result;
  result.vertical_in_base = vertical;
  result.base_to_level.row(0) = level_x.transpose();
  result.base_to_level.row(1) = level_y.transpose();
  result.base_to_level.row(2) = vertical.transpose();
  return result;
}

}  // namespace kilin_com_estimator
