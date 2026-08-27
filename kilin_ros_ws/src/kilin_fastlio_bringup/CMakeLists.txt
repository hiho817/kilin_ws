cmake_minimum_required(VERSION 3.8)
project(kilin_fastlio_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/hip_center_odometry_adapter.py
  scripts/run_terrain_shadow_replay.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_hip_center_odometry_adapter
    test/test_hip_center_odometry_adapter.py)
endif()

ament_package()
