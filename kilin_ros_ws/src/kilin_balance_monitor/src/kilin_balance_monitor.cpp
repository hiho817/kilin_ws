// Copyright 2026 Ian

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "kilin_msgs/msg/balance_state_stamped.hpp"
#include "kilin_msgs/msg/stability_state_stamped.hpp"
#include "kilin_msgs/msg/stair_phase_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
struct Point2
{
  double x{};
  double y{};
  double z{};
};

double cross(const Point2 & origin, const Point2 & a, const Point2 & b)
{
  return (a.x - origin.x) * (b.y - origin.y) -
         (a.y - origin.y) * (b.x - origin.x);
}

std::vector<Point2> convex_hull(std::vector<Point2> points)
{
  std::sort(
    points.begin(), points.end(),
    [](const Point2 & lhs, const Point2 & rhs) {
      return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
    });

  if (points.size() <= 2) {
    return points;
  }

  std::vector<Point2> hull;
  hull.reserve(points.size() * 2);
  for (const auto & point : points) {
    while (hull.size() >= 2 &&
      cross(hull[hull.size() - 2], hull.back(), point) <= 0.0)
    {
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
  return hull;
}

bool finite_point(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

const char * phase_name(int phase)
{
  switch (phase) {
    case 0:
      return "none";
    case 1:
      return "front-left";
    case 2:
      return "front-right";
    case 3:
      return "rear-left";
    case 4:
      return "rear-right";
    default:
      return "invalid";
  }
}
}  // namespace

class KilinBalanceMonitor : public rclcpp::Node
{
public:
  KilinBalanceMonitor()
  : Node("kilin_balance_monitor")
  {
    safe_margin_m_ = declare_parameter<double>("safe_margin_m", 0.03);
    log_period_sec_ = declare_parameter<double>("log_period_sec", 1.0);
    const auto balance_topic = declare_parameter<std::string>(
      "balance_state_topic", "/kilin/balance_state");
    const auto phase_topic = declare_parameter<std::string>(
      "stair_phase_topic", "/kilin/stair_phase");
    const auto stability_topic = declare_parameter<std::string>(
      "stability_state_topic", "/kilin/stability_state");
    if (safe_margin_m_ < 0.0 || log_period_sec_ <= 0.0) {
      throw std::runtime_error("safe_margin_m must be nonnegative and log_period_sec positive");
    }

    stability_pub_ = create_publisher<kilin_msgs::msg::StabilityStateStamped>(
      stability_topic, 10);
    balance_sub_ = create_subscription<kilin_msgs::msg::BalanceStateStamped>(
      balance_topic, 10,
      std::bind(&KilinBalanceMonitor::balance_callback, this, std::placeholders::_1));
    auto phase_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    phase_sub_ = create_subscription<kilin_msgs::msg::StairPhaseStamped>(
      phase_topic, phase_qos,
      std::bind(&KilinBalanceMonitor::phase_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Geometry balance monitor ready; safe margin %.3f m", safe_margin_m_);
  }

private:
  void phase_callback(const kilin_msgs::msg::StairPhaseStamped::SharedPtr msg)
  {
    if (msg->phase < 0 || msg->phase > 4) {
      RCLCPP_ERROR(get_logger(), "Ignoring invalid stair phase %d", msg->phase);
      return;
    }
    phase_ = msg->phase;
    waypoint_ = msg->waypoint;
    gait_paused_ = msg->gait_paused;
    have_phase_ = true;
    RCLCPP_INFO(
      get_logger(), "Phase %d (%s), waypoint=%s, gait_paused=%s",
      phase_, phase_name(phase_), waypoint_.c_str(), gait_paused_ ? "true" : "false");
  }

  void balance_callback(const kilin_msgs::msg::BalanceStateStamped::SharedPtr msg)
  {
    kilin_msgs::msg::StabilityStateStamped output;
    output.header = msg->header;
    output.phase = phase_;
    output.swing_leg = phase_name(phase_);
    output.required_margin = safe_margin_m_;
    output.com_projection.x = msg->com.x;
    output.com_projection.y = msg->com.y;

    if (!have_phase_ || !finite_point(msg->com)) {
      publish_invalid(output, "waiting for phase or finite COM");
      return;
    }

    std::vector<Point2> supports;
    supports.reserve(4);
    const int excluded_index = phase_ > 0 ? phase_ - 1 : -1;
    for (std::size_t index = 0; index < msg->contact_points.size(); ++index) {
      if (static_cast<int>(index) == excluded_index) {
        continue;
      }
      if (!finite_point(msg->contact_points[index])) {
        continue;
      }
      const auto & point = msg->contact_points[index];
      supports.push_back({point.x, point.y, point.z});
    }

    if (supports.size() < 3) {
      publish_invalid(output, "fewer than three finite geometric support points");
      return;
    }

    const auto hull = convex_hull(std::move(supports));
    if (hull.size() < 3) {
      publish_invalid(output, "support contacts are collinear");
      return;
    }

    double average_z = 0.0;
    for (const auto & point : hull) {
      average_z += point.z;
    }
    average_z /= static_cast<double>(hull.size());
    output.com_projection.z = average_z;

    for (const auto & point : hull) {
      geometry_msgs::msg::Point32 polygon_point;
      polygon_point.x = static_cast<float>(point.x);
      polygon_point.y = static_cast<float>(point.y);
      polygon_point.z = static_cast<float>(average_z);
      output.support_polygon.points.push_back(polygon_point);
    }

    double minimum_distance = std::numeric_limits<double>::infinity();
    double correction_x = 0.0;
    double correction_y = 0.0;
    const Point2 projected_com{msg->com.x, msg->com.y, average_z};
    for (std::size_t index = 0; index < hull.size(); ++index) {
      const auto & a = hull[index];
      const auto & b = hull[(index + 1) % hull.size()];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length = std::hypot(dx, dy);
      if (length <= 1e-9) {
        continue;
      }
      const double signed_distance = cross(a, b, projected_com) / length;
      if (signed_distance < minimum_distance) {
        minimum_distance = signed_distance;
        correction_x = -dy / length;
        correction_y = dx / length;
      }
    }

    if (!std::isfinite(minimum_distance)) {
      publish_invalid(output, "support polygon contains a zero-length edge");
      return;
    }

    output.valid = true;
    output.stability_margin = minimum_distance;
    output.inside = minimum_distance >= 0.0;
    output.safe = minimum_distance >= safe_margin_m_;
    output.correction_direction.x = correction_x;
    output.correction_direction.y = correction_y;
    output.correction_direction.z = 0.0;
    stability_pub_->publish(output);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), static_cast<int64_t>(log_period_sec_ * 1000.0),
      "phase=%d waypoint=%s supports=%zu margin=%.4f m inside=%s safe=%s",
      phase_, waypoint_.c_str(), hull.size(), minimum_distance,
      output.inside ? "true" : "false", output.safe ? "true" : "false");
  }

  void publish_invalid(
    kilin_msgs::msg::StabilityStateStamped & output, const char * reason)
  {
    output.valid = false;
    output.inside = false;
    output.safe = false;
    output.stability_margin = std::numeric_limits<double>::quiet_NaN();
    stability_pub_->publish(output);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), static_cast<int64_t>(log_period_sec_ * 1000.0),
      "Stability state invalid: %s", reason);
  }

  double safe_margin_m_{};
  double log_period_sec_{};
  int phase_{0};
  std::string waypoint_{"unknown"};
  bool gait_paused_{false};
  bool have_phase_{false};

  rclcpp::Subscription<kilin_msgs::msg::BalanceStateStamped>::SharedPtr balance_sub_;
  rclcpp::Subscription<kilin_msgs::msg::StairPhaseStamped>::SharedPtr phase_sub_;
  rclcpp::Publisher<kilin_msgs::msg::StabilityStateStamped>::SharedPtr stability_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KilinBalanceMonitor>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("kilin_balance_monitor"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
