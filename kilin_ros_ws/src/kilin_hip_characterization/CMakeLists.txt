cmake_minimum_required(VERSION 3.8)
project(kilin_hip_characterization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kilin_msgs REQUIRED)

add_executable(campaign_runner src/campaign_runner.cpp)
ament_target_dependencies(campaign_runner rclcpp kilin_msgs)

install(TARGETS campaign_runner DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/single_runner.py
  scripts/batch_runner.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_package()
