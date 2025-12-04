cmake_minimum_required(VERSION 3.8)
project(kilin_csv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kilin_msgs REQUIRED)

add_executable(kilin_csv_control src/kilin_csv_control.cpp)
ament_target_dependencies(kilin_csv_control
  rclcpp
  kilin_msgs
)

install(
  TARGETS kilin_csv_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY csv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
