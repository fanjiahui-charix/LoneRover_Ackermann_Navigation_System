cmake_minimum_required(VERSION 3.8)
project(origincarpro_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_core
  src/serial_port.cpp
  src/protocol.cpp
  src/imu_filter.cpp
  src/origincarpro_base.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

add_executable(origincarpro_base_node
  app/origincarpro_base_node.cpp
)

target_link_libraries(origincarpro_base_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(origincarpro_base_node
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}_core
  origincarpro_base_node
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
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  readme.cn.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()




# app/origincarpro_base_node.cpp
#         |
#         v
# origincarpro_base_node
#         |
#         links
#         |
#         v
# origincarpro_base_core
#         |
#         +-- serial_port.cpp
#         +-- protocol.cpp
#         +-- imu_filter.cpp
#         +-- origincarpro_base.cpp

# 文件	给谁看	作用
# package.xml	ROS2 / colcon / rosdep	知道包级依赖，决定构建顺序和安装依赖
# CMakeLists.txt	CMake / 编译器 / 链接器	找头文件、找库、链接目标

# package.xml：
#   “我这个包依赖 rclcpp”

# CMakeLists.txt：
#   “编译这个 target 的时候，把 rclcpp 的头文件和库链接进来”


# package.xml
#    |
#    |  colcon 读取包名、依赖、build_type
#    v
# CMakeLists.txt
#    |
#    |  find_package(...)
#    v
# ROS2 dependencies
#    |
#    v
# add_library(origincarpro_base_core)
#    |
#    +-- src/serial_port.cpp
#    +-- src/protocol.cpp
#    +-- src/imu_filter.cpp
#    +-- src/origincarpro_base.cpp
#    |
#    v
# add_executable(origincarpro_base_node)
#    |
#    +-- app/origincarpro_base_node.cpp
#    |
#    v
# target_link_libraries(origincarpro_base_node origincarpro_base_core)
#    |
#    v
# install/
#    |
#    +-- lib/origincarpro_base/origincarpro_base_node
#    +-- share/origincarpro_base/launch/
#    +-- share/origincarpro_base/config/


# 1. colcon 扫描工作区的 src 目录
# 2. 找到本包的 package.xml
# 3. 读取包名 origincarpro_base
# 4. 读取 build_type = ament_cmake
# 5. 知道这个包要用 CMake 构建
# 6. 创建 build/origincarpro_base/
# 7. 调用 cmake 配置阶段
# 8. CMake 读取 CMakeLists.txt
# 9. find_package(...) 找 ROS2 依赖
# 10. add_library(...) 定义 core 库
# 11. add_executable(...) 定义 node 可执行程序
# 12. 生成 Makefile 或 Ninja 构建脚本
# 13. 调用 make 或 ninja
# 14. g++ 编译 .cpp -> .o
# 15. linker 链接 .o -> .so / executable
# 16. install(...) 把结果安装到 install/origincarpro_base/
# 17. ament_package() 注册包索引
# 18. source install/setup.bash 后 ROS2 能找到这个包


# colcon build
#    |
#    v
# find package.xml
#    |
#    v
# read build_type: ament_cmake
#    |
#    v
# call CMake
#    |
#    v
# read CMakeLists.txt
#    |
#    +--> add_library(origincarpro_base_core)
#    |       |
#    |       +--> serial_port.cpp
#    |       +--> protocol.cpp
#    |       +--> imu_filter.cpp
#    |       +--> origincarpro_base.cpp
#    |
#    +--> add_executable(origincarpro_base_node)
#    |       |
#    |       +--> app/origincarpro_base_node.cpp
#    |
#    +--> link node with core
#    |
#    v
# install to install/origincarpro_base/
