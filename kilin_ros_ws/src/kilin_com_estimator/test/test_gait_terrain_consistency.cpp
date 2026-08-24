// Copyright 2026 Ian

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <tuple>

#include "kilin_com_estimator/robot_com_model.hpp"
#include "kilin_com_estimator/terrain_orientation.hpp"

namespace
{

TEST(GaitTerrainConsistency, ValidatedTenByThirtyFiveSupportRowsAreSolvable)
{
  kilin_com_estimator::RobotComModel model(TEST_URDF_PATH);
  using Case = std::tuple<
    std::array<double, 4>, std::array<int, 4>, std::array<bool, 4>>;
  const std::array<bool, 4> all = {true, true, true, true};
  const std::array<Case, 10> cases = {{
    {{0, 0, 0, 0}, {0, 0, 0, 0}, all},
    {{-40, -40, 40, 40}, {0, 0, 0, 0}, all},
    {{0, -40, 40, 40}, {0, 1, 0, 0}, all},
    {{320, -40, 40, 40}, {1, 1, 0, 0}, all},
    {{360, -40, 40, 40}, {1, 2, 0, 0}, all},
    {{400, -40, 40, 40}, {1, 2, 0, 0}, all},
    {{400, -40, 40, 320}, {1, 2, 0, 1}, all},
    {{400, -40, 320, 320}, {1, 2, 1, 1}, all},
    {{730, 10, 370, 370}, {2, 2, 1, 1}, all},
    {{730, 320, 380, 380}, {2, 3, 1, 1}, all},
  }};

  for (std::size_t case_index = 0; case_index < cases.size(); ++case_index) {
    std::map<std::string, double> joints;
    const auto & hips = std::get<0>(cases[case_index]);
    const std::array<std::string, 4> names = {"FL_hip", "FR_hip", "RL_hip", "RR_hip"};
    for (std::size_t leg = 0; leg < names.size(); ++leg) {
      joints[names[leg]] = hips[leg] * M_PI / 180.0;
    }
    const auto geometry = model.compute(joints);
    const auto orientation = kilin_com_estimator::estimate_terrain_orientation(
      geometry.wheel_centers, std::get<1>(cases[case_index]),
      std::get<2>(cases[case_index]), 0.10);
    EXPECT_TRUE(orientation.has_value()) << "case " << case_index;
  }
}

}  // namespace
