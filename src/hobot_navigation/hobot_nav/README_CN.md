# hobot_nav 中文说明

`hobot_nav` 是 ROS 2 导航集成包，负责把底盘反馈、EKF、二维雷达处理、Nav2、静态地图、代价地图、Tube/LUT 资产和 Ackermann 速度限制连接起来。

## 运行链路

```text
外部 LaserScan -> 滤波 -> 锥桶聚类 -> 锥桶代价地图层
轮速 + IMU -> ekf_fusion -> odom -> base_link
静态地图 -> Nav2 全局/局部 costmap
                    -> SmacPlannerHybrid -> RPP / 其他控制器
                    -> 速度平滑 -> Ackermann 限制 -> 底盘
```

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch hobot_nav navigation_core.launch.py \
  launch_base:=true \
  launch_lidar_pipeline:=true \
  launch_static_map:=true \
  enable_lidar_odom:=true
```

雷达输入默认是 `/scan_raw`。如果话题不同：

```bash
ros2 launch hobot_nav lidar_pipeline.launch.py \
  raw_scan_topic:=/my_lidar/scan \
  filtered_scan_topic:=/scan
```

这条启动链路使用已有静态地图，不负责在线建图。换车辆或换场地时，需要重新确认底盘标定、IMU、静态 TF、车辆 footprint、地图路径和最小转弯半径。

## 主要内容

- 轮速与 IMU EKF 定位；
- Smac Hybrid-A* 全局规划；
- 二维雷达滤波、去畸变、锥桶聚类和 costmap 图层；
- Nav2 Ackermann 行为树；
- 速度平滑、曲率限速和最终安全命令；
- 1 cm 地图、倒车入口 LUT、Tube 路径和虚拟车模调参。

## 主要接口

| 接口 | 作用 |
| --- | --- |
| `/odom/data_raw` | 原始轮式里程计 |
| `/imu/data_raw` | 原始 IMU |
| `/odom` | EKF 输出 |
| `/scan_raw` | 外部雷达输入 |
| `/scan` | 滤波后的雷达 |
| `/cones/points` | 锥桶聚类结果 |
| `/map` | 已有静态地图 |
| `/cmd_vel_nav` | Nav2 速度输出 |
| `/cmd_vel_safe` | 最终底盘速度命令 |

动态 `odom -> base_link` 由 `ekf_fusion` 发布；`base_link -> imu_link` 和 `base_link -> laser_link` 是静态外参。
