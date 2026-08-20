# lidar_local_planner 中文说明

`lidar_local_planner` 是一个基于二维 `LaserScan` 的轻量反应式局部避障参考节点：搜索可通行方向，根据前方距离减速、停止或转向，雷达数据超时则输出零速度。

它用于底盘和雷达联调，不负责全局规划、地图、定位融合、建图或 Nav2 路径跟踪。正式导航请使用 `hobot_nav` 的 Nav2 配置。

```bash
source /opt/ros/humble/setup.bash
ros2 launch lidar_local_planner local_scan_planner.launch.py \
  scan_topic:=/scan \
  cmd_vel_topic:=/cmd_vel
```

不要让本节点和 Nav2 同时向同一个底盘命令话题发布，除非已经配置命令复用或安全仲裁。
