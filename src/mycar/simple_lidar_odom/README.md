# simple_lidar_odom

这是一个基于二维 `LaserScan` 的轻量雷达里程计前端。它从滤波后的扫描中提取稳定
几何结构，估计平面位姿和速度，可作为 EKF 的可选观测源。

默认不发布 TF，避免与 `ekf_fusion` 争夺 `odom -> base_link`。在导航链路中通常使用：

```text
external LaserScan -> lidar_perception -> /scan
                                  -> simple_lidar_odom -> /lidar_odom
                                  -> ekf_fusion -> /odom
```

参数位于 `config/lidar_odom.yaml`。使用前确认雷达外参、扫描时间戳、`/odom` 和 IMU
话题正确。这个包不包含任何具体雷达驱动。

前端扫描滤波和雷达里程计应分开调试：先确认 `/scan` 的时间戳、角度和距离稳定，
再检查里程计输出。车辆 footprint、雷达外参和运动去畸变参数应来自同一套车模标定。
定位自转或路径抖动时，先排查输入扫描、时间戳与 TF 一致性，再调整规划和控制参数。
