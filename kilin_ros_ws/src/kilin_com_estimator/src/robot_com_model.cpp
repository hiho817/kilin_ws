// Copyright 2026 Ian

#include "kilin_com_estimator/robot_com_model.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

namespace kilin_com_estimator
{
namespace
{
constexpr double kAxisTolerance = 1e-12;
const std::array<std::string, 4> kWheelLinks = {
  "FL_wheel_link", "FR_wheel_link", "RL_wheel_link", "RR_wheel_link"};
}  // namespace

RobotComModel::RobotComModel(const std::string & urdf_path)
{
  if (!model_.initFile(urdf_path)) {
    throw std::runtime_error("failed to load URDF: " + urdf_path);
  }
  const auto initial = compute({});
  model_mass_ = initial.total_mass;
  if (!std::isfinite(model_mass_) || model_mass_ <= 0.0) {
    throw std::runtime_error("URDF contains no positive finite link mass");
  }
}

RobotComResult RobotComModel::compute(
  const std::map<std::string, double> & joint_positions) const
{
  const auto root = model_.getRoot();
  if (!root) {
    throw std::runtime_error("URDF has no root link");
  }

  Accumulator accumulator;
  traverse(root, Eigen::Isometry3d::Identity(), joint_positions, accumulator);
  if (!std::isfinite(accumulator.mass) || accumulator.mass <= 0.0) {
    throw std::runtime_error("computed robot mass is not positive and finite");
  }

  RobotComResult result;
  result.total_mass = accumulator.mass;
  result.com = accumulator.weighted_com / accumulator.mass;
  for (std::size_t index = 0; index < kWheelLinks.size(); ++index) {
    const auto found = accumulator.link_origins.find(kWheelLinks[index]);
    if (found == accumulator.link_origins.end()) {
      throw std::runtime_error("URDF lacks wheel link: " + kWheelLinks[index]);
    }
    result.wheel_centers[index] = found->second;
  }
  return result;
}

double RobotComModel::model_mass() const
{
  return model_mass_;
}

void RobotComModel::traverse(
  const urdf::LinkConstSharedPtr & link,
  const Eigen::Isometry3d & link_transform,
  const std::map<std::string, double> & joint_positions,
  Accumulator & accumulator) const
{
  accumulator.link_origins[link->name] = link_transform.translation();
  if (link->inertial && std::isfinite(link->inertial->mass) && link->inertial->mass > 0.0) {
    const Eigen::Vector3d local_com(
      link->inertial->origin.position.x,
      link->inertial->origin.position.y,
      link->inertial->origin.position.z);
    accumulator.mass += link->inertial->mass;
    accumulator.weighted_com +=
      link->inertial->mass * (link_transform * local_com);
  }

  for (const auto & child_joint : link->child_joints) {
    if (!child_joint) {
      continue;
    }
    const auto child_link = model_.getLink(child_joint->child_link_name);
    if (!child_link) {
      throw std::runtime_error("URDF joint has no child link: " + child_joint->name);
    }
    double position = 0.0;
    const auto found = joint_positions.find(child_joint->name);
    if (found != joint_positions.end()) {
      if (!std::isfinite(found->second)) {
        throw std::runtime_error("non-finite position for joint: " + child_joint->name);
      }
      position = found->second;
    }
    const Eigen::Isometry3d child_transform =
      link_transform * pose_transform(child_joint->parent_to_joint_origin_transform) *
      joint_motion(*child_joint, position);
    traverse(child_link, child_transform, joint_positions, accumulator);
  }
}

Eigen::Isometry3d RobotComModel::pose_transform(const urdf::Pose & pose)
{
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  pose.rotation.getQuaternion(qx, qy, qz, qw);
  Eigen::Quaterniond quaternion(qw, qx, qy, qz);
  quaternion.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

Eigen::Isometry3d RobotComModel::joint_motion(
  const urdf::Joint & joint, double position)
{
  Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
  if (joint.type == urdf::Joint::FIXED) {
    return motion;
  }

  Eigen::Vector3d axis(joint.axis.x, joint.axis.y, joint.axis.z);
  if (axis.norm() <= kAxisTolerance) {
    throw std::runtime_error("movable joint has a zero axis: " + joint.name);
  }
  axis.normalize();
  if (joint.type == urdf::Joint::REVOLUTE || joint.type == urdf::Joint::CONTINUOUS) {
    motion.linear() = Eigen::AngleAxisd(position, axis).toRotationMatrix();
  } else if (joint.type == urdf::Joint::PRISMATIC) {
    motion.translation() = position * axis;
  } else {
    throw std::runtime_error("unsupported URDF joint type for: " + joint.name);
  }
  return motion;
}

}  // namespace kilin_com_estimator
