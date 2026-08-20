// Copyright 2026 Ian
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KINOVA_JOINT_PTP__JOINT_MATH_HPP_
#define KINOVA_JOINT_PTP__JOINT_MATH_HPP_

#include <cmath>

namespace kinova_joint_ptp
{

inline double shortest_equivalent_target(double current, double target)
{
  constexpr double kTwoPi = 6.28318530717958647692;
  return current + std::remainder(target - current, kTwoPi);
}

inline double shortest_angular_error(double current, double target)
{
  constexpr double kTwoPi = 6.28318530717958647692;
  return std::remainder(target - current, kTwoPi);
}

}  // namespace kinova_joint_ptp

#endif  // KINOVA_JOINT_PTP__JOINT_MATH_HPP_
