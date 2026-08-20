cmake_minimum_required(VERSION 3.8)
project(simple_lidar_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)

add_library(simple_lidar_odom_core
  src/landmark_extractor.cpp
  src/local_landmark_map.cpp
  src/pose_covariance.cpp
  src/persistent_cone_map.cpp
  src/persistent_fence_map.cpp
  src/lidar_odom_policy.cpp
  src/unique_assignment.cpp
)
target_include_directories(simple_lidar_odom_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(simple_lidar_odom_core Eigen3::Eigen)
ament_target_dependencies(simple_lidar_odom_core sensor_msgs)

add_executable(lidar_odom_node src/lidar_odom_node.cpp)
target_include_directories(lidar_odom_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(lidar_odom_node simple_lidar_odom_core Eigen3::Eigen)
ament_target_dependencies(
  lidar_odom_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  tf2_ros
  tf2
  std_srvs
  visualization_msgs
)

if(BUILD_TESTING)
  ament_add_gtest(test_landmark_helpers
    test/test_landmark_helpers.cpp
  )
  if(TARGET test_landmark_helpers)
    target_link_libraries(test_landmark_helpers simple_lidar_odom_core Eigen3::Eigen)
  endif()

  ament_add_gtest(test_pose_covariance
    test/test_pose_covariance.cpp
  )
  if(TARGET test_pose_covariance)
    target_link_libraries(test_pose_covariance simple_lidar_odom_core Eigen3::Eigen)
  endif()

  ament_add_gtest(test_persistent_cone_map
    test/test_persistent_cone_map.cpp
  )
  if(TARGET test_persistent_cone_map)
    target_link_libraries(test_persistent_cone_map simple_lidar_odom_core Eigen3::Eigen)
  endif()

  ament_add_gtest(test_persistent_fence_map
    test/test_persistent_fence_map.cpp
  )
  if(TARGET test_persistent_fence_map)
    target_link_libraries(test_persistent_fence_map simple_lidar_odom_core Eigen3::Eigen)
  endif()

  ament_add_gtest(test_lidar_odom_policy
    test/test_lidar_odom_policy.cpp
  )
  if(TARGET test_lidar_odom_policy)
    target_link_libraries(test_lidar_odom_policy simple_lidar_odom_core Eigen3::Eigen)
  endif()

  ament_add_gtest(test_unique_assignment
    test/test_unique_assignment.cpp
  )
  if(TARGET test_unique_assignment)
    target_link_libraries(test_unique_assignment simple_lidar_odom_core Eigen3::Eigen)
  endif()
endif()

install(TARGETS simple_lidar_odom_core lidar_odom_node
  EXPORT export_simple_lidar_odom
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_simple_lidar_odom HAS_LIBRARY_TARGET)
ament_package()
