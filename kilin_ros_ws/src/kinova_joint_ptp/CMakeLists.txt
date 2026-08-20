cmake_minimum_required(VERSION 3.8)
project(kinova_joint_ptp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(kinova_ptp_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(kinova_joint_ptp_server_lib
  src/joint_ptp_server.cpp
)
target_compile_features(kinova_joint_ptp_server_lib PUBLIC cxx_std_17)
target_include_directories(kinova_joint_ptp_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(kinova_joint_ptp_server_lib
  control_msgs
  kinova_ptp_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

add_executable(kinova_joint_ptp_server src/main.cpp)
target_compile_features(kinova_joint_ptp_server PUBLIC cxx_std_17)
target_link_libraries(kinova_joint_ptp_server kinova_joint_ptp_server_lib)
ament_target_dependencies(kinova_joint_ptp_server rclcpp)

install(TARGETS
  kinova_joint_ptp_server_lib
  kinova_joint_ptp_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_joint_math test/test_joint_math.cpp)
  if(TARGET test_joint_math)
    target_compile_features(test_joint_math PUBLIC cxx_std_17)
    target_include_directories(test_joint_math PRIVATE include)
  endif()

  ament_add_gtest(test_joint_ptp_server test/test_joint_ptp_server.cpp)
  if(TARGET test_joint_ptp_server)
    target_compile_features(test_joint_ptp_server PUBLIC cxx_std_17)
    target_link_libraries(test_joint_ptp_server kinova_joint_ptp_server_lib)
    ament_target_dependencies(test_joint_ptp_server
      kinova_ptp_interfaces
      rclcpp
      rclcpp_action
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  control_msgs
  kinova_ptp_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

ament_package()
