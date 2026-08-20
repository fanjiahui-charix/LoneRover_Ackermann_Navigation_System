# LoneRover Ackermann Navigation

一个面向 Ackermann 小车的 ROS 2 导航系统，公开内容集中在定位、雷达感知、地图与代价地图、路径规划、局部跟踪、速度约束，以及虚实结合的离线调参工具。

`LoneRover` 的含义很简单：`Lone` 指这套导航链路由一个人独立搭建和打磨，`Rover` 指真正承载算法的车。它不是某个第三方框架的名称，而是这套工程的项目名。

[English README](README_EN.md)

## 公开范围

这个仓库是导航代码和导航资料的公开版本。任务层、视觉链路、模型文件、比赛专用启动入口，以及来源不明确的底层设备驱动不在仓库中。公开代码通过标准 ROS 2 话题与外部底盘、雷达驱动连接，因此不会把第三方代码或比赛私有依赖伪装成可直接复现的组件。

保留下来的重点是可以独立学习、测试和复用的导航部分：

- 轮速与 IMU 的 EKF 惯导融合；
- 外部 `LaserScan` 输入的滤波、去躁、锥桶聚类与 Nav2 代价地图图层；
- 1 cm 静态地图、静态障碍层、动态障碍层、锥桶层、禁行区域和膨胀层；
- `SmacPlannerHybrid` 全局规划。它本身就是面向 Ackermann 车辆的 Hybrid-A* 规划器；
- 普通区域的 Nav2 路径跟踪、窄通道的 Tube 路径与 RPP 跟踪；
- 基于真实车模的倒车入口 LUT、虚拟车辆延迟模型和离线评估工具；
- 速度平滑、曲率限速、横向加速度约束、倒车限速和安全命令输出。

## 系统结构

```text
底盘串口 / 轮速 / IMU
          │
          ▼
origincarpro_base ──► ekf_fusion ──► odom -> base_link
          │                         │
          └──► 外部静态 TF          └──► map 下的 Nav2 导航

外部雷达驱动 ──► /scan_raw ──► 滤波 / 去躁 ──► /scan
                                      │
                                      └──► 锥桶聚类 ──► 锥桶代价地图层

静态地图 ───────────────────────────────► 全局 costmap
静态地图 + /scan + 锥桶层 + 禁区层 ─────► 局部 costmap

任务层提供目标或路径
          │
          ▼
SmacPlannerHybrid ──► 路径 ──► RPP / 其他 Nav2 控制器
                                      │
                                      ▼
                         速度平滑与 Ackermann 约束
                                      │
                                      ▼
                              /cmd_vel_safe
```

任务层负责选择目标、切换路线和决定何时使用 Tube 或 LUT；公开仓库只保留导航执行侧。Nav2 的行为树负责导航 action 内部的规划、跟踪、重规划和恢复，不承担未公开的比赛任务决策。

## 核心技术

### EKF 惯导定位

`ekf_fusion` 使用底盘反馈的轮速信息和 IMU 信息进行松耦合融合。松耦合的含义是：轮速里程计和 IMU 先各自形成标准观测，再由 EKF 统一更新状态；每种传感器的驱动、消息格式和误差模型保持相对独立。

这套定位的关键不在于堆叠传感器，而在于对底盘反馈做标定：轮速比例、转向比例、IMU 零偏、时间戳、车体坐标系和转弯时的观测噪声都需要与真实车辆一致。滤波器只发布动态的 `odom -> base_link`，静态地图关系由地图侧或起始位姿侧提供，避免多个节点同时抢占同一条 TF。

公开配置保留了正常运行和保守运行两种参数入口，便于先验证原始反馈，再逐步调整过程噪声和观测噪声。仓库不包含录包专用流程，离线调参工具只依赖用户自己准备的 ROS 2 数据。

### 地图与代价地图

比赛场地是固定环境，公开版本使用已有的 1 cm 静态栅格地图，不包含建图启动流程。地图同时承担导航坐标、规划坐标和障碍物坐标的统一管理。

工程中也试过 5 mm 甚至 1 mm 的栅格。分辨率变细会明显增加地图、代价地图、规划和可视化的计算量，而车辆尺寸、雷达噪声和定位误差并没有同步变小。对这台车而言，1 cm 在精度、实时性和可维护性之间更合适。

全局代价地图使用静态地图和必要的障碍信息；局部代价地图实时叠加雷达障碍、锥桶图层、禁行区域和膨胀层。锥桶图层只接收聚类后的几何点，不依赖相机或模型。

### 全局规划与局部跟踪

普通区域和倒车入口连接使用 `SmacPlannerHybrid`。这里的 Smac 与 Hybrid-A* 不是两套规划器，`SmacPlannerHybrid` 就是 Nav2 中的 Hybrid-A* 实现，负责在车辆最小转弯半径、方向和 footprint 约束下生成可行路径。

规划器输出的是 `Path`，局部控制器负责跟踪这条路径并输出速度。公开默认配置使用 RPP；在窄通道中，规划重点从“不断搜索导航点”改成“跟踪预先验证过的 Tube 路径”。因此，路径和规划器的职责是分开的：路径描述车辆应该经过哪里，规划器负责搜索或连接路径，控制器负责在运行中跟踪并处理局部误差。

### Tube 通道路径

窄通道里连续堆叠导航点会让全局规划和状态交接变得脆弱，也很难稳定处理边界障碍。公开配置保留三种横向路径：`center`、`inner`、`outer`，每种路径都有顺、逆两个方向。运行时可以依据当前障碍和安全余量选择一条路径，再由 RPP 跟踪。

Tube 的优势是几何约束明确、切换逻辑简单、在线计算量小。路径生成和边界检查放在离线阶段，运行时只做候选选择、当前代价地图复核和安全输出。这种方式也方便把窄通道的避障从大量导航点调整，变成少量经过验证的路径模板。

### 倒车入口 LUT 与虚实结合调参

倒车入口 LUT 不是凭经验写几个目标点，而是使用车上实际的 Ackermann 车辆模型逐个位姿生成和验证候选。候选会检查最小转弯半径、车身 footprint、地图障碍和入口连接关系，在线根据当前位姿查表，再用当前代价地图做一次复核。

离线调参同时保留两种车辆模型：一种调用真实车上的规划配置，另一种是带有转向响应延迟、速度响应延迟和限幅的虚拟车模。先在虚拟车上比较 Tube、RPP、速度和延迟参数，再到实体车做少量确认，可以显著减少反复捡车和盲调参数的次数。相关工具位于 `tools/` 和 `tools/channel_tuning/`。

### 速度控制与安全输出

速度控制不是规划器的附属参数。公开代码会根据曲率、横向加速度上限、前后向速度上限、加速度和减速度限制生成平滑速度，并在命令输出侧做 Ackermann 半径、倒车方向和过期命令检查。最终底盘只接收经过限制器处理的 `/cmd_vel_safe`。

## 目录结构

```text
src/hobot_navigation/
├── hobot_nav/              # Nav2 启动、参数、代价地图、Tube、导航工具
├── lidar_perception/        # LaserScan 滤波、锥桶聚类、锥桶代价地图层
├── lidar_local_planner/     # 雷达局部规划辅助模块
└── lidar_web_viewer/        # 标准雷达话题的轻量查看器

src/mycar/
├── origincarpro_base/       # 底盘串口、轮速、IMU 和基础 TF 接口
├── ekf_fusion/              # 轮速 + IMU EKF 惯导
├── simple_lidar_odom/       # 外部雷达里程计接口与辅助实现
├── mycar_navigation/        # 聚类、路径和速度控制辅助模块
└── origincar_description/   # 含响应延迟的虚拟 Ackermann 车模

tools/                       # LUT、Tube、RPP、shadow vehicle 离线工具
docs/                        # 依赖、公开边界和调参说明
```

## 系统要求

- Ubuntu 22.04；
- ROS 2 Humble；
- Nav2 及其标准插件；
- 一台能够提供轮速、IMU 和 Ackermann 控制接口的底盘；
- 一个由用户自行安装的雷达驱动，向本仓库提供 `sensor_msgs/msg/LaserScan`；
- 一张与车辆 footprint、坐标系和起始位姿一致的静态地图。

雷达驱动没有放在这里。原因是车上的驱动来源不完全明确，而且驱动本身属于设备适配层；只要它发布标准 `/scan_raw`，就可以替换。X5 交叉编译使用仓库外部准备好的工具链，不把平台 SDK、系统包或厂商构建仓库复制进来。

依赖说明见 [`docs/DEPENDENCIES.md`](docs/DEPENDENCIES.md)，公开边界见 [`docs/OPEN_SOURCE_SCOPE.md`](docs/OPEN_SOURCE_SCOPE.md)。

## 构建与运行

在已经安装 ROS 2 Humble 和 Nav2 的环境中：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ros_workspace_open_source
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
```

启动公开的导航链路：

```bash
ros2 launch hobot_nav navigation_core.launch.py
```

默认启动静态地图、底盘接口、EKF、雷达处理、Nav2 和安全速度输出。若外部雷达驱动发布的话题不是 `/scan_raw`，只需重映射：

```bash
ros2 launch hobot_nav navigation_core.launch.py \
  raw_scan_topic:=/my_lidar/scan \
  enable_lidar_odom:=false
```

这条命令面向导航组件验证，不包含完整比赛任务入口。使用自有底盘时，需要先让底盘节点发布 `/odom/data_raw`、`/imu/data_raw`，并订阅 `/cmd_vel_safe`。

## 主要话题与 TF

| 方向 | 接口 | 说明 |
|---|---|---|
| 输入 | `/odom/data_raw` | 底盘原始轮速里程计 |
| 输入 | `/imu/data_raw` | 底盘 IMU |
| 输入 | `/scan_raw` | 外部雷达驱动的原始 LaserScan |
| 输出 | `/odom` | EKF 融合后的里程计 |
| 输出 | `/scan` | 滤波后的 LaserScan |
| 输出 | `/cones/points` | 锥桶聚类后的几何点 |
| 输出 | `/map` | 静态 OccupancyGrid |
| 输出 | `/plan`、`/local_plan` | 全局路径和局部跟踪路径 |
| 输出 | `/cmd_vel_nav` | Nav2 速度命令 |
| 输出 | `/cmd_vel_safe` | 限制器处理后的底盘命令 |

核心 TF 为 `map -> odom -> base_link`，传感器静态外参包括 `base_link -> imu_link` 和 `base_link -> laser_link`。相机 TF 和视觉话题不属于公开运行链路。

## 离线工具

- `tools/generate_reverse_gate_lut.py`：生成倒车入口候选；
- `tools/validate_reverse_gate_paths.py`：检查候选路径的曲率、footprint 和地图可行性；
- `tools/offline_mppi_shadow_sim.py`：在给定路径和虚拟车辆参数下进行离线跟踪评估；
- `tools/channel_tuning/`：Tube、连接段、RPP 和锥桶避障的几何生成与参数扫描；
- `hobot_nav/scripts/waypoint_audit.py`：检查导航点文件的坐标、朝向和间距；
- `hobot_nav/scripts/nav_doctor.py`：检查导航运行所需的基本话题、TF 和参数。

离线工具不会自动寻找录包，也不会依赖仓库内的录包数据。请把个人数据文件放在仓库之外，并通过参数显式传入。

## 调试建议

可以用 Foxglove 观察话题频率、TF、路径和代价地图；大地图场景下 RViz 的交互开销通常更高。定位异常时优先查看原始轮速、IMU、EKF 输出和地图坐标关系；如果车辆出现自转或规划路径抖动，应分别检查定位误差、曲率约束、局部控制器和速度限制，而不是只修改一个导航点。

窄通道调试时，减少连续导航点，优先验证 Tube 几何、RPP 横向误差、障碍膨胀半径和切换余量。参数先在虚拟车辆和离线路径上筛选，再进行实体车验证。

## 许可证与第三方依赖

本仓库只对公开目录中的自有代码和说明负责。ROS 2、Nav2、底盘平台包以及雷达驱动分别遵循各自的许可证和发布方式；使用前请阅读 [`docs/DEPENDENCIES.md`](docs/DEPENDENCIES.md)。
