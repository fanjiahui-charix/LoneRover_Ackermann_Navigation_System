# hobot_navigation

这是公开导航代码的 ROS 2 包目录。当前只保留自己维护的导航集成、雷达处理、局部
规划和网页调试工具：

- `hobot_nav`：Nav2 参数、静态地图、行为树、Tube/LUT、代价地图入口和速度限制；
- `lidar_perception`：外部二维雷达输入的滤波、锥桶聚类和代价地图层；
- `lidar_local_planner`：轻量雷达局部规划参考实现；
- `lidar_web_viewer`：只查看 `LaserScan` 的低负载网页工具；
- `adaptive_speed_limiter`：曲率、障碍物间距和目标距离相关的速度限制。

雷达串口驱动、相机/视觉、二维码、大模型、BEV、地瓜/TROS 源码和平台 SDK 均不在
此目录。使用者需要自行准备兼容的 ROS 2 雷达驱动，让它发布 `/scan_raw`。

启动入口是：

```bash
ros2 launch hobot_nav navigation_core.launch.py
```

具体话题、TF、构建依赖和运行边界见
[`hobot_nav/README_CN.md`](hobot_nav/README_CN.md)。英文说明见
[`hobot_nav/README.md`](hobot_nav/README.md)。
