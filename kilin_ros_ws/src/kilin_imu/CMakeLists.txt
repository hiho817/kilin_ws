cmake_minimum_required(VERSION 3.8)
project(kilin_imu)

# set(CMAKE_CXX_STANDARD 14)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(gpiod REQUIRED)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(GPIOD REQUIRED libgpiod)

# include_directories(${GPIOD_INCLUDE_DIRS})
# link_directories(${GPIOD_LIBRARY_DIRS})




# # # Include mip_sdk headers
# add_subdirectory(
#   ${CMAKE_CURRENT_SOURCE_DIR}/../mip_sdk
#   ${CMAKE_CURRENT_BINARY_DIR}/mip_sdk_build
# )



# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Add mip_sdk as a subdirectory
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../mip_sdk)
add_subdirectory(src/mip_sdk)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/mip_sdk/src)
# include_directories(src/mip_sdk/include)

set(MIP_UTILS "${CMAKE_CURRENT_LIST_DIR}/src/mip_utils")


set(DEVICE_SOURCES
    "${MIP_UTILS}/mip_utils.c"
    "${MIP_UTILS}/mip_utils.h"
    "${MIP_UTILS}/mip_utils.cpp"
    "${MIP_UTILS}/mip_utils.hpp"
)

if(MIP_USE_SERIAL)
    set(SERIAL_DEFS "MIP_USE_SERIAL")
endif()

if(MIP_USE_TCP)
    set(TCP_DEFS "MIP_USE_TCP")
endif()

if(MIP_USE_EXTRAS)
    set(EXTRAS_DEFS "MIP_USE_EXTRAS")
endif()

set(MIP_DEFS ${MIP_DEFINES} ${SERIAL_DEFS} ${TCP_DEFS} ${EXTRAS_DEFS})

# MIP_DeviceInfo
add_executable(MIP_DeviceInfo src/mip_device_info.cpp ${DEVICE_SOURCES})
target_link_libraries(MIP_DeviceInfo mip "${SERIAL_LIB}" "${SOCKET_LIB}")
target_compile_definitions(MIP_DeviceInfo PUBLIC "${MIP_DEFS}")

# MIP_WatchImu
add_executable(MIP_WatchImu src/mip_watch_imu.cpp ${DEVICE_SOURCES})
target_link_libraries(MIP_WatchImu mip "${SERIAL_LIB}" "${SOCKET_LIB}")
target_compile_definitions(MIP_WatchImu PUBLIC "${MIP_DEFS}")


# MIP_CX5_GX5_CV5_15_25_Example
add_executable(MIP_IMU_LOGGER src/imu_logger.cpp ${DEVICE_SOURCES})
target_link_libraries(MIP_IMU_LOGGER mip "${SERIAL_LIB}" "${SOCKET_LIB}" ${CMAKE_CURRENT_SOURCE_DIR}/src/mip_sdk/build/libmip.a)
target_compile_definitions(MIP_IMU_LOGGER PUBLIC "${MIP_DEFS}")
ament_target_dependencies(MIP_IMU_LOGGER rclcpp)


# imu_publisher.cpp
add_executable(IMUpublisher src/imu_publisher.cpp ${DEVICE_SOURCES})
target_link_libraries(IMUpublisher mip "${SERIAL_LIB}" "${SOCKET_LIB}" ${CMAKE_CURRENT_SOURCE_DIR}/src/mip_sdk/build/libmip.a)
target_compile_definitions(IMUpublisher PUBLIC "${MIP_DEFS}")
ament_target_dependencies(IMUpublisher rclcpp sensor_msgs Eigen3)
# add_executable(IMUpublisher src/imu_publisher.cpp ${DEVICE_SOURCES})
# target_link_libraries(IMUpublisher )
# target_compile_definitions(IMUpublisher PUBLIC)
# ament_target_dependencies(IMUpublisher rclcpp std_msgs sensor_msgs)

# imu_subscriber_logger.cpp
add_executable(imu_subscriber_logger src/imu_subscriber_logger.cpp)
ament_target_dependencies(imu_subscriber_logger rclcpp sensor_msgs std_srvs)
find_library(GPIOD_LIB gpiod)
target_link_libraries(imu_subscriber_logger ${GPIOD_LIB})





install(
  TARGETS MIP_IMU_LOGGER IMUpublisher imu_subscriber_logger  
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)
# install(TARGETS imu_subscriber_logger DESTINATION lib/${PROJECT_NAME})
# install(TARGETS MIP_IMU_LOGGER DESTINATION lib/${PROJECT_NAME})
# install(TARGETS IMUpublisher DESTINATION lib/${PROJECT_NAME})



ament_package()