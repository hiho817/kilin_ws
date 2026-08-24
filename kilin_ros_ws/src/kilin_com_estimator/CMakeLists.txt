cmake_minimum_required(VERSION 3.8)
project(kilin_com_estimator)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kilin_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(robot_com_model src/robot_com_model.cpp src/joint_mapping.cpp)
target_compile_features(robot_com_model PUBLIC cxx_std_17)
target_include_directories(
  robot_com_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_com_model Eigen3::Eigen)
ament_target_dependencies(robot_com_model urdf)

add_executable(kilin_com_estimator src/kilin_com_estimator.cpp)
target_compile_features(kilin_com_estimator PUBLIC cxx_std_17)
target_link_libraries(kilin_com_estimator robot_com_model Eigen3::Eigen)
ament_target_dependencies(
  kilin_com_estimator
  ament_index_cpp
  geometry_msgs
  kilin_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS robot_com_model kilin_com_estimator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_com_model test/test_robot_com_model.cpp)
  target_link_libraries(test_robot_com_model robot_com_model Eigen3::Eigen)
  target_compile_definitions(
    test_robot_com_model PRIVATE
    TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../kilin_robot_description/urdf/kilin_gen3_hardware.urdf"
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(robot_com_model)
ament_export_dependencies(Eigen3 urdf)
ament_package()
