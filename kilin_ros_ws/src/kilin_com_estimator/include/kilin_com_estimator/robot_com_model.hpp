// Copyright 2026 Ian

#ifndef KILIN_COM_ESTIMATOR__ROBOT_COM_MODEL_HPP_
#define KILIN_COM_ESTIMATOR__ROBOT_COM_MODEL_HPP_

#include <Eigen/Geometry>
#include <urdf/model.h>

#include <array>
#include <map>
#include <string>

namespace kilin_com_estimator
{

struct RobotComResult
{
  Eigen::Vector3d com{Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 4> wheel_centers{};
  double total_mass{0.0};
};

class RobotComModel
{
public:
  explicit RobotComModel(const std::string & urdf_path);

  RobotComResult compute(const std::map<std::string, double> & joint_positions) const;
  double model_mass() const;

private:
  struct Accumulator
  {
    double mass{0.0};
    Eigen::Vector3d weighted_com{Eigen::Vector3d::Zero()};
    std::map<std::string, Eigen::Vector3d> link_origins;
  };

  void traverse(
    const urdf::LinkConstSharedPtr & link,
    const Eigen::Isometry3d & link_transform,
    const std::map<std::string, double> & joint_positions,
    Accumulator & accumulator) const;

  static Eigen::Isometry3d pose_transform(const urdf::Pose & pose);
  static Eigen::Isometry3d joint_motion(
    const urdf::Joint & joint, double position);

  urdf::Model model_;
  double model_mass_{0.0};
};

}  // namespace kilin_com_estimator

#endif  // KILIN_COM_ESTIMATOR__ROBOT_COM_MODEL_HPP_
