cmake_minimum_required(VERSION 3.8)
project(kilin_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kilin_msgs REQUIRED)

add_executable(kilin_logger src/kilin_logger.cpp)
ament_target_dependencies(kilin_logger
  rclcpp
  kilin_msgs
)

install(TARGETS
  kilin_logger
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
