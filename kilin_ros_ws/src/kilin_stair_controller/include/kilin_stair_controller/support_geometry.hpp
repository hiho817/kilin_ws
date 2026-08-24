// Copyright 2026 Ian

#ifndef KILIN_STAIR_CONTROLLER__SUPPORT_GEOMETRY_HPP_
#define KILIN_STAIR_CONTROLLER__SUPPORT_GEOMETRY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace kilin_stair_controller
{
namespace geometry
{
struct Point2
{
  double x{};
  double y{};
};

struct StabilityResult
{
  std::vector<Point2> hull;
  std::vector<Point2> safe_polygon;
  double signed_margin{};
  bool inside_support{};
  bool inside_safe_region{};
  Point2 target;
  Point2 correction;
  Point2 direction;
  double correction_distance{};
};

inline double arm_joint1_for_direction(
  const Point2 & direction_in_amr, double arm_base_yaw_offset_rad)
{
  if (!std::isfinite(direction_in_amr.x) || !std::isfinite(direction_in_amr.y) ||
    !std::isfinite(arm_base_yaw_offset_rad) ||
    std::hypot(direction_in_amr.x, direction_in_amr.y) <= 1e-12)
  {
    throw std::runtime_error(
            "Arm direction must be finite and nonzero, and base-yaw offset must be finite");
  }
  return arm_base_yaw_offset_rad - std::atan2(direction_in_amr.y, direction_in_amr.x);
}

inline double cross(const Point2 & origin, const Point2 & a, const Point2 & b)
{
  return (a.x - origin.x) * (b.y - origin.y) -
         (a.y - origin.y) * (b.x - origin.x);
}

inline double signed_area(const std::vector<Point2> & polygon)
{
  double twice_area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    twice_area += a.x * b.y - b.x * a.y;
  }
  return 0.5 * twice_area;
}

inline std::vector<Point2> convex_hull(std::vector<Point2> points)
{
  if (points.size() < 3) {
    throw std::runtime_error("At least three support points are required");
  }
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::runtime_error("Support points must be finite");
    }
  }
  std::sort(
    points.begin(), points.end(), [](const Point2 & lhs, const Point2 & rhs) {
      return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
    });
  points.erase(
    std::unique(
      points.begin(), points.end(), [](const Point2 & lhs, const Point2 & rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y;
      }),
    points.end());
  if (points.size() < 3) {
    throw std::runtime_error("Support points do not form a polygon");
  }

  std::vector<Point2> hull;
  hull.reserve(points.size() * 2);
  for (const auto & point : points) {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const std::size_t lower_size = hull.size();
  for (auto iterator = points.rbegin() + 1; iterator != points.rend(); ++iterator) {
    while (hull.size() > lower_size &&
      cross(hull[hull.size() - 2], hull.back(), *iterator) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(*iterator);
  }
  hull.pop_back();
  if (hull.size() < 3 || signed_area(hull) <= 1e-12) {
    throw std::runtime_error("Support points are collinear");
  }
  return hull;
}

inline std::vector<double> signed_edge_distances(
  const Point2 & point, const std::vector<Point2> & polygon)
{
  std::vector<double> distances;
  distances.reserve(polygon.size());
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length <= 1e-12) {
      throw std::runtime_error("Support polygon contains a zero-length edge");
    }
    distances.push_back(((point.x - a.x) * -dy + (point.y - a.y) * dx) / length);
  }
  return distances;
}

inline bool inside_polygon(const Point2 & point, const std::vector<Point2> & polygon)
{
  const auto distances = signed_edge_distances(point, polygon);
  return *std::min_element(distances.begin(), distances.end()) >= -1e-10;
}

inline std::vector<Point2> inset_polygon(
  const std::vector<Point2> & hull, double margin)
{
  if (!std::isfinite(margin) || margin < 0.0) {
    throw std::runtime_error("Safety margin must be finite and nonnegative");
  }
  if (margin == 0.0) {
    return hull;
  }

  std::vector<Point2> directions;
  std::vector<Point2> shifted;
  directions.reserve(hull.size());
  shifted.reserve(hull.size());
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const auto & a = hull[i];
    const auto & b = hull[(i + 1) % hull.size()];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    directions.push_back({dx / length, dy / length});
    shifted.push_back({a.x - margin * dy / length, a.y + margin * dx / length});
  }

  std::vector<Point2> inset;
  inset.reserve(hull.size());
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const std::size_t previous = (i + hull.size() - 1) % hull.size();
    const auto & a = shifted[previous];
    const auto & da = directions[previous];
    const auto & b = shifted[i];
    const auto & db = directions[i];
    const double denominator = da.x * db.y - da.y * db.x;
    if (std::abs(denominator) <= 1e-12) {
      throw std::runtime_error("Cannot inset polygon with parallel adjacent edges");
    }
    const double parameter = ((b.x - a.x) * db.y - (b.y - a.y) * db.x) / denominator;
    inset.push_back({a.x + parameter * da.x, a.y + parameter * da.y});
  }
  if (signed_area(inset) <= 1e-12) {
    throw std::runtime_error("Safety margin leaves no valid support region");
  }
  for (const auto & point : inset) {
    const auto distances = signed_edge_distances(point, hull);
    if (*std::min_element(distances.begin(), distances.end()) < margin - 1e-9) {
      throw std::runtime_error("Safety margin leaves no valid support region");
    }
  }
  return inset;
}

inline Point2 closest_point_on_segment(
  const Point2 & point, const Point2 & start, const Point2 & end)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1e-18) {
    return start;
  }
  const double unbounded = ((point.x - start.x) * dx + (point.y - start.y) * dy) /
    length_squared;
  const double parameter = std::clamp(unbounded, 0.0, 1.0);
  return {start.x + parameter * dx, start.y + parameter * dy};
}

inline Point2 closest_point_on_polygon(
  const Point2 & point, const std::vector<Point2> & polygon)
{
  if (inside_polygon(point, polygon)) {
    return point;
  }
  Point2 closest{};
  double minimum_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto candidate = closest_point_on_segment(
      point, polygon[i], polygon[(i + 1) % polygon.size()]);
    const double distance = std::hypot(candidate.x - point.x, candidate.y - point.y);
    if (distance < minimum_distance) {
      minimum_distance = distance;
      closest = candidate;
    }
  }
  return closest;
}

inline StabilityResult evaluate_stability(
  const Point2 & com, const std::array<Point2, 4> & wheel_points, int phase,
  double safety_margin)
{
  if (phase < 1 || phase > 4 || !std::isfinite(com.x) || !std::isfinite(com.y)) {
    throw std::runtime_error("COM must be finite and phase must be 1 through 4");
  }
  std::vector<Point2> supports;
  supports.reserve(3);
  for (std::size_t i = 0; i < wheel_points.size(); ++i) {
    if (static_cast<int>(i) != phase - 1) {
      supports.push_back(wheel_points[i]);
    }
  }
  const auto hull = convex_hull(std::move(supports));
  const auto distances = signed_edge_distances(com, hull);
  const double signed_margin = *std::min_element(distances.begin(), distances.end());
  const auto safe_polygon = inset_polygon(hull, safety_margin);
  const bool inside_safe = inside_polygon(com, safe_polygon);
  const auto target = inside_safe ? com : closest_point_on_polygon(com, safe_polygon);
  const Point2 correction{target.x - com.x, target.y - com.y};
  const double correction_distance = std::hypot(correction.x, correction.y);
  const Point2 direction = correction_distance > 1e-12 ?
    Point2{correction.x / correction_distance, correction.y / correction_distance} : Point2{};
  return {
    hull, safe_polygon, signed_margin, signed_margin >= -1e-10, inside_safe,
    target, correction, direction, correction_distance};
}
}  // namespace geometry
}  // namespace kilin_stair_controller

#endif  // KILIN_STAIR_CONTROLLER__SUPPORT_GEOMETRY_HPP_
