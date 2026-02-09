cmake_minimum_required(VERSION 3.8)
project(kilin_cmd_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kilin_msgs REQUIRED)

# Executable
add_executable(kilin_cmd_converter src/kilin_cmd_converter.cpp)
ament_target_dependencies(kilin_cmd_converter
  rclcpp
  std_msgs
  geometry_msgs
  kilin_msgs
)

install(TARGETS
  kilin_cmd_converter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
