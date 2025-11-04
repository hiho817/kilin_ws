cmake_minimum_required(VERSION 3.8)
project(kilin_joystick)

# 強制 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_PREFIX_PATH "/opt/ros/humble;${LOCAL_PACKAGE_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(kilin_joystick src/kilin_joystick.cpp)

ament_target_dependencies(kilin_joystick
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  kilin_joystick
  DESTINATION lib/${PROJECT_NAME})

# 明確 export
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs)
ament_package()
