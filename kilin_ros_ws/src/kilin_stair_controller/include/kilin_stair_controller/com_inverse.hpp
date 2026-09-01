// Copyright 2026 Ian

#ifndef KILIN_STAIR_CONTROLLER__COM_INVERSE_HPP_
#define KILIN_STAIR_CONTROLLER__COM_INVERSE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "kilin_stair_controller/support_geometry.hpp"

namespace kilin_stair_controller
{
namespace inverse
{
using Matrix3 = std::array<std::array<double, 3>, 3>;

struct Result
{
  double alpha{};
  bool reachable{};
  bool already_safe{};
  bool saturated{};
  geometry::Point2 predicted_delta;
  geometry::Point2 predicted_com;
  double predicted_safe_margin{};
};

inline constexpr std::array<double, 6> DXY_COEFFICIENTS_M =
{0.0, 0.01194366, 0.17659841, 0.00485676, -0.19429831, 0.07956216};
inline constexpr std::array<double, 5> DZ_COEFFICIENTS_M =
{0.0, 0.11049534, 0.00072290, -0.24782693, 0.12700811};

template<std::size_t SizeT>
inline double polynomial(const std::array<double, SizeT> & coefficients, double alpha)
{
  double value = 0.0;
  for (auto iterator = coefficients.rbegin(); iterator != coefficients.rend(); ++iterator) {
    value = value * alpha + *iterator;
  }
  return value;
}

inline double fitted_dxy(double alpha)
{
  if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
    throw std::runtime_error("alpha must be finite and in [0, 1]");
  }
  return polynomial(DXY_COEFFICIENTS_M, alpha);
}

inline double fitted_dz(double alpha)
{
  if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
    throw std::runtime_error("alpha must be finite and in [0, 1]");
  }
  return polynomial(DZ_COEFFICIENTS_M, alpha);
}

inline double feedforward_target_margin(double release_margin, double margin_offset)
{
  if (!std::isfinite(release_margin) || release_margin < 0.0 ||
    !std::isfinite(margin_offset) || margin_offset < 0.0)
  {
    throw std::runtime_error(
            "release margin and inverse margin offset must be finite and nonnegative");
  }
  return std::max(0.0, release_margin - margin_offset);
}

inline Matrix3 identity_rotation()
{
  return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}

inline geometry::Point2 predicted_world_delta(
  double alpha, const geometry::Point2 & extension_direction_base,
  const Matrix3 & base_to_output_rotation)
{
  const double direction_norm = std::hypot(
    extension_direction_base.x, extension_direction_base.y);
  if (!std::isfinite(direction_norm) || direction_norm <= 1e-12) {
    throw std::runtime_error("extension direction must be finite and nonzero");
  }
  for (const auto & row : base_to_output_rotation) {
    for (const double value : row) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("base-to-output rotation must be finite");
      }
    }
  }
  const double dxy = fitted_dxy(alpha);
  const double dz = fitted_dz(alpha);
  const double body_x = dxy * extension_direction_base.x / direction_norm;
  const double body_y = dxy * extension_direction_base.y / direction_norm;
  return {
    base_to_output_rotation[0][0] * body_x +
    base_to_output_rotation[0][1] * body_y +
    base_to_output_rotation[0][2] * dz,
    base_to_output_rotation[1][0] * body_x +
    base_to_output_rotation[1][1] * body_y +
    base_to_output_rotation[1][2] * dz};
}

inline double safe_margin(
  const geometry::Point2 & point, const std::vector<geometry::Point2> & safe_polygon)
{
  const auto distances = geometry::signed_edge_distances(point, safe_polygon);
  return *std::min_element(distances.begin(), distances.end());
}

inline Result solve_minimum_alpha(
  const geometry::Point2 & current_com, const std::vector<geometry::Point2> & safe_polygon,
  const geometry::Point2 & extension_direction_base, const Matrix3 & base_to_output_rotation,
  double minimum_alpha = 0.0, double scan_step = 0.001)
{
  if (!std::isfinite(current_com.x) || !std::isfinite(current_com.y)) {
    throw std::runtime_error("current COM must be finite");
  }
  if (safe_polygon.size() < 3 || geometry::signed_area(safe_polygon) <= 1e-12) {
    throw std::runtime_error("safe polygon must be finite, nondegenerate, and counter-clockwise");
  }
  if (!std::isfinite(minimum_alpha) || minimum_alpha < 0.0 || minimum_alpha > 1.0 ||
    !std::isfinite(scan_step) || scan_step <= 0.0 || scan_step > 1.0)
  {
    throw std::runtime_error("minimum alpha and inverse scan step are invalid");
  }

  auto evaluate = [&](double alpha) {
      const auto delta = predicted_world_delta(
        alpha, extension_direction_base, base_to_output_rotation);
      const geometry::Point2 predicted{current_com.x + delta.x, current_com.y + delta.y};
      return Result{
      alpha, false, false, false, delta, predicted, safe_margin(predicted, safe_polygon)};
    };

  const double initial_margin = safe_margin(current_com, safe_polygon);
  if (initial_margin >= -1e-10 && minimum_alpha <= 1e-12) {
    auto result = evaluate(0.0);
    result.reachable = true;
    result.already_safe = true;
    return result;
  }

  double previous_alpha = minimum_alpha;
  auto previous = evaluate(previous_alpha);
  if (previous.predicted_safe_margin >= -1e-10) {
    previous.reachable = true;
    return previous;
  }

  for (double upper_alpha = std::min(1.0, minimum_alpha + scan_step);;
    upper_alpha = std::min(1.0, upper_alpha + scan_step))
  {
    auto upper = evaluate(upper_alpha);
    if (upper.predicted_safe_margin >= -1e-10) {
      double lower_bound = previous_alpha;
      double upper_bound = upper_alpha;
      for (int iteration = 0; iteration < 60 && upper_bound - lower_bound > 1e-9; ++iteration) {
        const double midpoint = 0.5 * (lower_bound + upper_bound);
        if (evaluate(midpoint).predicted_safe_margin >= -1e-10) {
          upper_bound = midpoint;
        } else {
          lower_bound = midpoint;
        }
      }
      auto result = evaluate(upper_bound);
      result.reachable = true;
      result.saturated = std::abs(upper_bound - 1.0) <= 1e-9;
      return result;
    }
    previous_alpha = upper_alpha;
    previous = upper;
    if (upper_alpha >= 1.0 - 1e-12) {
      break;
    }
  }

  auto result = evaluate(1.0);
  result.saturated = true;
  return result;
}
}  // namespace inverse
}  // namespace kilin_stair_controller
#endif  // KILIN_STAIR_CONTROLLER__COM_INVERSE_HPP_
