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
find_package(SDL2 REQUIRED)

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

add_executable(kilin_keyboard src/kilin_keyboard.cpp)
ament_target_dependencies(kilin_keyboard rclcpp geometry_msgs)
target_link_libraries(kilin_keyboard SDL2::SDL2)

# ----------------------------------------
# Install targets
# ----------------------------------------
install(TARGETS
  kilin_joystick
  kilin_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rclcpp sensor_msgs std_msgs geometry_msgs)
ament_package()
