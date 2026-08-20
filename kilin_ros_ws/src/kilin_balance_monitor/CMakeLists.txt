cmake_minimum_required(VERSION 3.8)
project(kilin_balance_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kilin_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(kilin_balance_monitor src/kilin_balance_monitor.cpp)
target_compile_features(kilin_balance_monitor PUBLIC cxx_std_17)
ament_target_dependencies(
  kilin_balance_monitor
  geometry_msgs
  kilin_msgs
  rclcpp
)

install(TARGETS kilin_balance_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
