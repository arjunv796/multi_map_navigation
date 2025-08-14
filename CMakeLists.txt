cmake_minimum_required(VERSION 3.8)
project(multi_map_navigation)

# --------------------
# Dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)
find_package(rosidl_default_generators REQUIRED)

# --------------------
# Generate custom action messages
# --------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToMap.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# --------------------
# Build component library
# --------------------
add_library(multi_map_server SHARED src/multi_map_server.cpp)
ament_target_dependencies(multi_map_server
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  geometry_msgs
)

# Link generated typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(multi_map_server "${cpp_typesupport_target}" ${SQLITE3_LIBRARIES})

# Include dirs for library
target_include_directories(multi_map_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SQLITE3_INCLUDE_DIRS}
)

# Register as component
rclcpp_components_register_nodes(multi_map_server "multi_map_navigation::MultiMapServer")

# --------------------
# Standalone executable (with main.cpp)
# --------------------
add_executable(multi_map_server_node
  src/multi_map_server_main.cpp
  src/multi_map_server.cpp
)
ament_target_dependencies(multi_map_server_node
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  geometry_msgs
)
target_link_libraries(multi_map_server_node "${cpp_typesupport_target}" ${SQLITE3_LIBRARIES})

target_include_directories(multi_map_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SQLITE3_INCLUDE_DIRS}
)

# --------------------
# Install
# --------------------
install(TARGETS
  multi_map_server
  multi_map_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

