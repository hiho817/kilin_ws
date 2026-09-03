cmake_minimum_required(VERSION 3.8)
project(kilin_hip_characterization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kilin_msgs REQUIRED)

add_library(hip_control_strategy SHARED src/hip_control_strategy.cpp)
target_include_directories(hip_control_strategy PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

add_executable(campaign_runner src/campaign_runner.cpp)
ament_target_dependencies(campaign_runner rclcpp kilin_msgs)
target_link_libraries(campaign_runner hip_control_strategy)

add_executable(fine_tune_processor src/fine_tune_processor.cpp)
ament_target_dependencies(fine_tune_processor rclcpp kilin_msgs)
target_link_libraries(fine_tune_processor hip_control_strategy)

install(TARGETS campaign_runner fine_tune_processor hip_control_strategy EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
install(PROGRAMS
  scripts/single_runner.py
  scripts/batch_runner.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_package()
