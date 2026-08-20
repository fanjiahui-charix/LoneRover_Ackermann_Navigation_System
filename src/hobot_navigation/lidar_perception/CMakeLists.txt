cmake_minimum_required(VERSION 3.8)
project(lidar_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

include_directories(include)
add_executable(scan_filter_node
  src/scan_filter_node.cpp
)
ament_target_dependencies(scan_filter_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

add_executable(cone_detector_node
  src/cone_detector_node.cpp
)

add_library(cone_layer SHARED
  src/cone_layer.cpp
  src/gate_side_soft_layer.cpp
)
ament_target_dependencies(cone_layer
  geometry_msgs
  map_msgs
  nav_msgs
  nav2_msgs
  nav2_costmap_2d
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_target_dependencies(cone_detector_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
add_executable(scan_deskew_node
  src/scan_deskew_node.cpp
)
ament_target_dependencies(scan_deskew_node
  nav_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS
  scan_filter_node
  cone_detector_node
  scan_deskew_node
  cone_layer
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d cone_layer_plugin.xml)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
