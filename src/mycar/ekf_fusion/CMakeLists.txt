cmake_minimum_required(VERSION 3.8)
project(ekf_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(angles REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPose.srv"
  "srv/ToggleFilterProcessing.srv"
  DEPENDENCIES
    geometry_msgs
)

add_library(${PROJECT_NAME}_core
  src/ekf.cpp
  src/filter_base.cpp
  src/filter_utilities.cpp
  src/ros_filter.cpp
  src/ros_filter_utilities.cpp
  src/lidar_odom_safety.cpp
  src/real_vehicle_events.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  angles
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_core "${cpp_typesupport_target}")

add_executable(ekf_fusion_node
  src/ekf_node.cpp
)

target_link_libraries(ekf_fusion_node
  ${PROJECT_NAME}_core
)

if(BUILD_TESTING)
  ament_add_gtest(test_lidar_odom_safety
    test/test_lidar_odom_safety.cpp
  )
  if(TARGET test_lidar_odom_safety)
    target_link_libraries(test_lidar_odom_safety ${PROJECT_NAME}_core)
  endif()
endif()

ament_target_dependencies(ekf_fusion_node
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}_core
  ekf_fusion_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "rdk_2026_hospital_static.pgm" EXCLUDE
  PATTERN "rdk_2026_hospital_static.yaml" EXCLUDE
)
install(CODE "
  file(REMOVE
    \"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/maps/rdk_2026_hospital_static.pgm\"
    \"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/maps/rdk_2026_hospital_static.yaml\"
  )
")

install(PROGRAMS
  scripts/sequence_cmd_vel.py
  scripts/static_map_publisher.py
  scripts/timed_cmd_vel.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  LICENSE
  README.md
  README_TUNING.md
  README_VEHICLE_TEST.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
