# Navigation dependencies

公开包通过标准 ROS 2 接口连接外部设备和系统组件。

| Dependency | Source |
|---|---|
| ROS 2 Humble | target operating-system installation |
| Nav2 | system package or the [official Navigation2 repository](https://github.com/ros-navigation/navigation2) |
| Ackermann chassis | external chassis package providing wheel odometry, IMU, TF, and command output |
| 2D lidar | external driver publishing `/scan_raw` as `sensor_msgs/msg/LaserScan` |
| static map | user-provided map consistent with the vehicle frame and footprint |

The lidar driver is intentionally outside this repository. This keeps the public tree independent of a particular device vendor and lets users replace the driver as long as the topic and frame contract remains stable.
