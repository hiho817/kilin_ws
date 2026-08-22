cmake_minimum_required(VERSION 3.8)
project(kilin_stair_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kilin_msgs REQUIRED)
find_package(kinova_ptp_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

add_executable(kilin_stair_controller src/kilin_stair_controller.cpp)
target_compile_features(kilin_stair_controller PUBLIC cxx_std_17)
target_include_directories(kilin_stair_controller PRIVATE include)
ament_target_dependencies(kilin_stair_controller
  geometry_msgs
  kilin_msgs
  kinova_ptp_interfaces
  rclcpp
  rclcpp_action
)

target_include_directories(kilin_stair_controller PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
target_link_libraries(kilin_stair_controller ${LIBGPIOD_LIBRARIES})
target_compile_options(kilin_stair_controller PRIVATE ${LIBGPIOD_CFLAGS_OTHER})

install(TARGETS kilin_stair_controller DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/generate_stair_csv.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_stair_csv
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # package.xml still has a TODO license, so do not claim a license here.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_support_geometry test/test_support_geometry.cpp)
  if(TARGET test_support_geometry)
    target_compile_features(test_support_geometry PUBLIC cxx_std_17)
    target_include_directories(test_support_geometry PRIVATE include)
  endif()

  ament_add_pytest_test(test_generate_stair_csv test/test_generate_stair_csv.py)
endif()

ament_package()
