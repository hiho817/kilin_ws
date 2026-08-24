// Copyright 2026 Ian

#ifndef KILIN_COM_ESTIMATOR__JOINT_MAPPING_HPP_
#define KILIN_COM_ESTIMATOR__JOINT_MAPPING_HPP_

#include <map>
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
  std::string & error);

double actual_motor_position(double position, double position_diff);

}  // namespace kilin_com_estimator

#endif  // KILIN_COM_ESTIMATOR__JOINT_MAPPING_HPP_
