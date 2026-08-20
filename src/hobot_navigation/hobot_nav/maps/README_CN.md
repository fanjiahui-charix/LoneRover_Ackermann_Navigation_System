# 静态地图中文说明

导航包通过 `nav2_map_server` 加载已有静态地图，运行链路不包含在线建图流程。

当前保留的 1 cm 地图同时用于导航和离线调参。5 mm、1 mm 地图曾经用于测试，但会增加内存和规划开销，超过车辆 footprint 和传感器误差能够提供的实际收益。

可以使用 `static_map.launch.py` 单独发布地图，也可以使用 `navigation_core.launch.py` 启动完整导航链路。
