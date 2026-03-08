cmake_minimum_required(VERSION 3.8)
project(kilin_csv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kilin_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

add_executable(kilin_csv_control src/kilin_csv_control.cpp)

ament_target_dependencies(kilin_csv_control
  rclcpp
  kilin_msgs
)

target_include_directories(kilin_csv_control PRIVATE
  ${LIBGPIOD_INCLUDE_DIRS}
)

target_link_libraries(kilin_csv_control
  ${LIBGPIOD_LIBRARIES}
)

target_compile_options(kilin_csv_control PRIVATE
  ${LIBGPIOD_CFLAGS_OTHER}
)

install(
  TARGETS kilin_csv_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY csv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()