// Copyright 2026 Ian

#ifndef KILIN_COM_ESTIMATOR__COM_BIAS_HPP_
#define KILIN_COM_ESTIMATOR__COM_BIAS_HPP_

#include <Eigen/Core>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace kilin_com_estimator
{

inline Eigen::Vector3d parse_com_bias(const std::vector<double> & values)
{
  if (values.size() != 3U) {
    throw std::invalid_argument("com_bias_base_m must contain exactly 3 values");
  }
  if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
    !std::isfinite(values[2]))
  {
    throw std::invalid_argument("com_bias_base_m values must be finite");
  }
  return {values[0], values[1], values[2]};
}

}  // namespace kilin_com_estimator

#endif  // KILIN_COM_ESTIMATOR__COM_BIAS_HPP_
