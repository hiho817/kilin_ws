cmake_minimum_required(VERSION 3.8)
project(kilin_hip_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Allow FetchContent
include(FetchContent)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kilin_msgs REQUIRED)

# Fetch cpp-httplib (header-only)
FetchContent_Declare(cpp-httplib
  URL https://github.com/yhirose/cpp-httplib/archive/refs/tags/v0.11.0.zip
)
FetchContent_GetProperties(cpp-httplib)
if(NOT cpp-httplib_POPULATED)
  FetchContent_Populate(cpp-httplib)
endif()

# Fetch nlohmann_json (header-only)
FetchContent_Declare(json
  URL https://github.com/nlohmann/json/releases/download/v3.11.2/include.zip
)
FetchContent_GetProperties(json)
if(NOT json_POPULATED)
  FetchContent_Populate(json)
endif()

# Executable
add_executable(kilin_hip_controller src/kilin_hip_controller.cpp)
target_include_directories(kilin_hip_controller PRIVATE 
  ${cpp-httplib_SOURCE_DIR}
  ${json_SOURCE_DIR}/include
)
ament_target_dependencies(kilin_hip_controller
  rclcpp
  std_msgs
  kilin_msgs
)

# Install executable
install(TARGETS
  kilin_hip_controller
  DESTINATION lib/${PROJECT_NAME})

# Install web resources
install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
