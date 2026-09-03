cmake_minimum_required(VERSION 3.8)
project(kilin_hip_characterization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kilin_msgs REQUIRED)
# ROS Humble's CMake exports the classic Python discovery helpers used by the
# distro pybind11 package; avoid its newer Python3_add_library mode.
set(PYBIND11_FINDPYTHON OFF)
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 REQUIRED)

add_library(hip_control_strategy SHARED src/hip_control_strategy.cpp)
target_include_directories(hip_control_strategy PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

add_executable(campaign_runner src/campaign_runner.cpp)
ament_target_dependencies(campaign_runner rclcpp kilin_msgs)
target_link_libraries(campaign_runner hip_control_strategy)

pybind11_add_module(_hip_control_strategy src/hip_control_strategy_pybind.cpp)
target_link_libraries(_hip_control_strategy PRIVATE hip_control_strategy)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(TARGETS campaign_runner hip_control_strategy EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(TARGETS _hip_control_strategy LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
install(PROGRAMS
  scripts/single_runner.py
  scripts/batch_runner.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_package()
