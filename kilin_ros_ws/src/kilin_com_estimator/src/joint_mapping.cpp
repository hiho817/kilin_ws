// Copyright 2026 Ian

#include "kilin_com_estimator/joint_mapping.hpp"

#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace kilin_com_estimator
{

bool map_named_arm_positions(
  const std::vector<std::string> & names,
  const std::vector<double> & positions,
  const std::vector<std::string> & input_joint_names,
  const std::vector<std::string> & urdf_joint_names,
  std::map<std::string, double> & mapped_positions,
  std::string & error)
{
  mapped_positions.clear();
  error.clear();
  if (input_joint_names.size() != urdf_joint_names.size()) {
    error = "input and URDF arm joint-name lists have different sizes";
    return false;
  }
  if (names.size() > positions.size()) {
    error = "JointState has fewer positions than names";
    return false;
  }

  std::map<std::string, double> received;
  for (std::size_t index = 0; index < names.size(); ++index) {
    received[names[index]] = positions[index];
  }

  std::vector<std::string> missing;
  for (std::size_t index = 0; index < input_joint_names.size(); ++index) {
    const auto found = received.find(input_joint_names[index]);
    if (found == received.end() || !std::isfinite(found->second)) {
      missing.push_back(input_joint_names[index]);
      continue;
    }
    mapped_positions[urdf_joint_names[index]] = found->second;
  }

  if (!missing.empty()) {
    std::ostringstream stream;
    stream << "missing or non-finite Kinova joints:";
    for (const auto & name : missing) {
      stream << ' ' << name;
    }
    error = stream.str();
    mapped_positions.clear();
    return false;
  }
  return true;
}

double actual_motor_position(double position, double position_diff)
{
  return position + position_diff;
}

}  // namespace kilin_com_estimator
