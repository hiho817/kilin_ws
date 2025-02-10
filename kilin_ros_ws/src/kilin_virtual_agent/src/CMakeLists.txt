# Find required custom libraries
find_library(GRPC_PROTO_LIB grpc_proto_lib)
if (NOT GRPC_PROTO_LIB)
  message(FATAL_ERROR "Not found GRPC_PROTO Library.")
endif()

add_executable(kilin_virtual_agent kilin_virtual_agent.cpp)

ament_target_dependencies(kilin_virtual_agent rclcpp std_msgs rosgraph_msgs geometry_msgs kilin_msgs)

target_include_directories(kilin_virtual_agent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kilin_virtual_agent PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(kilin_virtual_agent
  ${CMAKE_DL_LIBS}
  ${GRPC_PROTO_LIB}
  ${_CORE_LIBRARIES}  # Optional: Only include if this variable is defined
)
  
target_link_libraries(kilin_virtual_agent
  ${CMAKE_DL_LIBS}
  ${GRPC_PROTO_LIB}
  ${_CORE_LIBRARIES}  # Optional: Only include if this variable is defined
)

install(TARGETS kilin_virtual_agent
  DESTINATION lib/${PROJECT_NAME})