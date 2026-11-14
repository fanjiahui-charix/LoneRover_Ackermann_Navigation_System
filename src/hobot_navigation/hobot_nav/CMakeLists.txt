cmake_minimum_required(VERSION 3.8)
project(hobot_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(gate_region_reachability STATIC
  src/gate_region_reachability.cpp
)
target_compile_features(gate_region_reachability PUBLIC cxx_std_17)
target_include_directories(gate_region_reachability PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gate_region_reachability
  geometry_msgs nav2_msgs nav_msgs)

add_executable(ackermann_command_limiter
  src/ackermann_command_limiter.cpp
)
target_compile_features(ackermann_command_limiter PUBLIC cxx_std_17)
target_include_directories(ackermann_command_limiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ackermann_command_limiter
  geometry_msgs rclcpp std_msgs)

option(BUILD_COMPETITION_SUPERVISOR "Build the competition mission coordinator" OFF)
if(BUILD_COMPETITION_SUPERVISOR)
  find_package(hbmem_wechatcode_msgs REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(ollama_msgs REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_library(qr_lane_reference STATIC src/qr_lane_reference.cpp)
  target_compile_features(qr_lane_reference PUBLIC cxx_std_17)
  target_include_directories(qr_lane_reference PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  add_executable(competition_supervisor
    src/competition_supervisor_main.cpp
    src/competition_supervisor.cpp
    src/motion_action_arbiter.cpp)
  target_compile_features(competition_supervisor PUBLIC cxx_std_17)
  target_include_directories(competition_supervisor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(competition_supervisor
    gate_region_reachability qr_lane_reference)
  ament_target_dependencies(competition_supervisor
    ament_index_cpp geometry_msgs hbmem_wechatcode_msgs lifecycle_msgs
    nav2_msgs nav_msgs ollama_msgs rclcpp rclcpp_action sensor_msgs std_msgs
    tf2 tf2_ros)

  add_executable(competition_command_limiter src/competition_command_limiter.cpp)
  target_compile_features(competition_command_limiter PUBLIC cxx_std_17)
  ament_target_dependencies(competition_command_limiter geometry_msgs rclcpp std_msgs)
  install(TARGETS competition_supervisor competition_command_limiter
    DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_gate_region_reachability
    test/test_gate_region_reachability.cpp
  )
  if(TARGET test_gate_region_reachability)
    target_link_libraries(test_gate_region_reachability gate_region_reachability)
    ament_target_dependencies(
      test_gate_region_reachability geometry_msgs nav2_msgs nav_msgs)
  endif()

  ament_add_pytest_test(
    test_native_mppi_cone_clearance_evaluator
    test/test_native_mppi_cone_clearance_evaluator.py
  )
  ament_add_pytest_test(
    test_native_mppi_shadow_cones_contract
    test/test_native_mppi_shadow_cones_contract.py
  )
endif()

install(
  TARGETS ackermann_command_limiter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/nav2_startup_guard.py
    scripts/nav_doctor.py
    scripts/check_lidar_pipeline.sh
    scripts/publish_static_frames.py
    scripts/waypoint_audit.py
    scripts/waypoint_recorder.py
    scripts/generate_map_registered_tubes.py
    scripts/fixed_start_tf.py
    scripts/fixed_start_goal.py
    scripts/competition_publish_static_frames.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/fixed_start_utils.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/fixed_start_utils.py
  DESTINATION share/${PROJECT_NAME}/scripts)

install(
  DIRECTORY launch config maps rviz waypoints behavior_trees runtime
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES
    README.md
    README_NAV_TUNING.md
    README_CHANNEL_MODES.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
