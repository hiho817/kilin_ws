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

#include <gtest/gtest.h>

#include <cmath>

#include "kinova_joint_ptp/joint_math.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

TEST(JointMath, KeepsNearbyTarget)
{
  EXPECT_NEAR(kinova_joint_ptp::shortest_equivalent_target(0.2, 0.5), 0.5, 1e-12);
}

TEST(JointMath, WrapsPositiveTargetAcrossPi)
{
  const double current = 170.0 * kPi / 180.0;
  const double requested = -170.0 * kPi / 180.0;
  const double expected = 190.0 * kPi / 180.0;
  EXPECT_NEAR(
    kinova_joint_ptp::shortest_equivalent_target(current, requested), expected, 1e-12);
}

TEST(JointMath, WrapsNegativeTargetAcrossPi)
{
  const double current = -170.0 * kPi / 180.0;
  const double requested = 170.0 * kPi / 180.0;
  const double expected = -190.0 * kPi / 180.0;
  EXPECT_NEAR(
    kinova_joint_ptp::shortest_equivalent_target(current, requested), expected, 1e-12);
}

TEST(JointMath, MeasuresEquivalentAnglesAsZeroError)
{
  const double current = -170.0 * kPi / 180.0;
  const double target = 190.0 * kPi / 180.0;
  EXPECT_NEAR(kinova_joint_ptp::shortest_angular_error(current, target), 0.0, 1e-12);
}
