# 导航点中文说明

本目录提供一个用于测试 Nav2 的通用导航点示例，不包含完整比赛任务路线或传感器任务逻辑。

`example_waypoints.yaml` 是 `waypoint_audit.py` 和 `waypoint_recorder.py` 使用的格式。替换成自己地图中的位姿后，先检查坐标、朝向和点间距，再发送给 Nav2。

```bash
ros2 run hobot_nav waypoint_audit.py \
  --waypoints-file src/hobot_navigation/hobot_nav/waypoints/example_waypoints.yaml \
  --route-name default
```
