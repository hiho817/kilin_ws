cmake_minimum_required(VERSION 3.8)
project(kilin_joystick)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ----------------------------------------
# Build executable
# ----------------------------------------
add_executable(kilin_joystick src/kilin_joystick.cpp)

ament_target_dependencies(kilin_joystick
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

# ----------------------------------------
# Install targets
# ----------------------------------------
install(TARGETS
  kilin_joystick
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rclcpp sensor_msgs std_msgs geometry_msgs)
ament_package()
