# lidar_local_planner

一个基于二维 `LaserScan` 的轻量反应式局部避障参考节点。它从雷达扇区选择相对开阔的方向，依据前方距离减速、停止或转向，并在数据超时时输出零速度。

它适合底盘与雷达联调，也可以作为简单安全测试工具；它不负责全局路径、地图、SLAM、定位融合或替代 Nav2 controller。正式地图导航使用 `hobot_nav` 中的 Nav2 配置。

## Quick start

```bash
source /opt/ros/humble/setup.bash
ros2 launch lidar_local_planner local_scan_planner.launch.py \
  scan_topic:=/scan \
  cmd_vel_topic:=/cmd_vel
```

输入默认是滤波后的 `/scan`，输出默认是 `/cmd_vel`。不要让它与 Nav2 同时直接写入同一个底盘命令话题，除非外部已经配置命令复用或安全仲裁。
