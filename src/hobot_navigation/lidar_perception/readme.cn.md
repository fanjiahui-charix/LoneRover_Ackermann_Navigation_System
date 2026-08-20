# lidar_perception

这是与厂商驱动解耦的二维雷达处理包，不包含串口驱动。任何 ROS 2 雷达驱动只要
发布合法时间戳、正确的 `LaserScan` 和雷达坐标系，就可以把数据接到 `/scan_raw`。

处理链：

```text
/scan_raw -> scan_filter_node -> /scan -> cone_detector_node
                                      -> /cones/points、/cones/poses、/cones/markers
```

包内包含距离/角度标定、中值/跳变/孤立点滤波、可选基于 `/odom` 的一阶去畸变、
锥桶聚类与时间确认，以及写入 Nav2 代价地图的锥桶层插件。

启动示例：

```bash
ros2 launch hobot_nav lidar_pipeline.launch.py \
  raw_scan_topic:=/my_lidar/scan \
  filtered_scan_topic:=/scan
```

参数集中在 `config/lidar_perception.yaml`。先标定距离比例、偏置和角度零点，再调
孤立点与聚类阈值。只有在 Nav2 代价地图启用锥桶层后，锥桶结果才会影响规划。
