# Navigation scripts

这些脚本服务于公开导航组件的检查和离线准备，不包含比赛任务层和设备驱动。

- `nav2_startup_guard.py`：等待 Nav2 基础节点达到可用状态；
- `nav_doctor.py`：检查话题、TF、地图和参数；
- `check_lidar_pipeline.sh`：检查外部雷达到 `/scan` 的处理链；
- `publish_static_frames.py`：发布 IMU 和雷达静态外参；
- `waypoint_audit.py`：检查导航点坐标、朝向和间距；
- `waypoint_recorder.py`：从标准 RViz 输入记录导航点；
- `generate_map_registered_tubes.py`：生成与地图坐标绑定的 Tube 路径。

这些工具不会启动相机、模型、任务状态机或雷达驱动。请把个人数据和录包放在仓库外部，通过命令行参数显式传入。
