cmake_minimum_required(VERSION 3.8)
project(mycar_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavGoal.msg"
  "msg/GoalStatus.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(nav_core
  src/nav_core/field_odom_transform.cpp
  src/nav_core/footprint.cpp
  src/nav_core/arc.cpp
  src/nav_core/map_mask.cpp
)

add_library(planner
  src/planner/local_grid.cpp
  src/planner/feasibility.cpp
  src/planner/speed_limiter.cpp
  src/planner/scorer.cpp
  src/planner/planner.cpp
)

add_library(clustering
  src/clustering/cluster_extractor.cpp
  src/clustering/cone_memory.cpp
)

add_library(navigator
  src/navigator/scan_conversion.cpp
  src/navigator/navigator_core.cpp
)

target_include_directories(nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(clustering PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(nav_core PUBLIC cxx_std_17)
target_compile_features(planner PUBLIC cxx_std_17)
target_compile_features(clustering PUBLIC cxx_std_17)
target_compile_features(navigator PUBLIC cxx_std_17)
target_link_libraries(nav_core yaml-cpp)
target_link_libraries(planner nav_core)
target_link_libraries(clustering nav_core)
target_link_libraries(navigator planner nav_core)

add_executable(local_navigator_node
  src/local_navigator_node.cpp
)

ament_target_dependencies(local_navigator_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  rosidl_default_runtime
)
target_link_libraries(local_navigator_node navigator ${cpp_typesupport_target})

install(TARGETS
  nav_core
  planner
  clustering
  navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  local_navigator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_nav_core
    test/nav_core/test_field_odom_transform.cpp
    test/nav_core/test_footprint.cpp
    test/nav_core/test_arc.cpp
    test/nav_core/test_map_mask.cpp
  )

  if(TARGET test_nav_core)
    target_link_libraries(test_nav_core nav_core)
  endif()

  ament_add_gtest(test_planner
    test/planner/test_local_grid.cpp
    test/planner/test_feasibility.cpp
    test/planner/test_speed_limiter.cpp
    test/planner/test_scorer.cpp
    test/planner/test_planner.cpp
  )

  if(TARGET test_planner)
    target_link_libraries(test_planner planner nav_core)
  endif()

  ament_add_gtest(test_clustering
    test/clustering/test_cluster_extractor.cpp
    test/clustering/test_cone_memory.cpp
    test/clustering/test_iron_law.cpp
  )

  if(TARGET test_clustering)
    target_link_libraries(test_clustering clustering planner nav_core)
  endif()

  ament_add_gtest(test_navigator
    test/navigator/test_scan_conversion.cpp
    test/navigator/test_navigator_core.cpp
  )

  if(TARGET test_navigator)
    target_link_libraries(test_navigator navigator planner nav_core)
  endif()
endif()

install(DIRECTORY
  launch
  config
  maps
  msg
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

ament_export_include_directories(include)
ament_export_libraries(nav_core planner clustering navigator)
ament_export_dependencies(yaml-cpp)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
