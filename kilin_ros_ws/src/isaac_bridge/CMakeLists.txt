cmake_minimum_required(VERSION 3.8)
project(isaac_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kilin_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 定義執行檔
add_executable(isaac_converter src/isaac_converter.cpp)
# 連結依賴庫
ament_target_dependencies(isaac_converter 
  rclcpp 
  sensor_msgs 
  kilin_msgs
  std_msgs
)

# 安裝設定 (讓 ros2 run 找得到)
install(TARGETS
  isaac_converter
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
