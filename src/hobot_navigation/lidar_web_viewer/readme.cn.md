# lidar_web_viewer 中文说明

`lidar_web_viewer` 订阅标准 `sensor_msgs/msg/LaserScan` 话题，提供一个轻量网页查看器。它不包含雷达驱动，也不依赖相机、模型或建图节点。

```bash
ros2 launch lidar_web_viewer lidar_web_viewer.launch.py
```

浏览器打开 `http://<device-ip>:8765`。输入话题和坐标系参数以包内 YAML 为准。
