# 导航点中文说明

本目录提供 Nav2 通用导航点示例、`fixed_start_points.yaml` 固定起点，以及比赛与演示导航点配置。

`example_waypoints.yaml` 是 `waypoint_audit.py` 和 `waypoint_recorder.py` 使用的格式。替换成自己地图中的位姿后，先检查坐标、朝向和点间距，再发送给 Nav2。

```bash
ros2 run hobot_nav waypoint_audit.py \
  --waypoints-file src/hobot_navigation/hobot_nav/waypoints/example_waypoints.yaml \
  --route-name default
```
