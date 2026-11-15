# Navigation scripts

这些脚本用于导航启动检查、坐标对齐、目标发送和离线准备。

- `nav2_startup_guard.py`：等待 Nav2 基础节点达到可用状态；
- `nav_doctor.py`：检查话题、TF、地图和参数；
- `check_lidar_pipeline.sh`：检查外部雷达到 `/scan` 的处理链；
- `publish_static_frames.py`：发布 IMU 和雷达静态外参；
- `competition_publish_static_frames.py`：发布比赛配置中的 IMU、雷达及可选相机静态外参；
- `fixed_start_tf.py`：根据固定起点和稳定里程计计算并发布 `map -> odom`；
- `fixed_start_goal.py`：检查导航就绪状态，发送固定起点流程的 QR 区导航目标并等待结果；
- `waypoint_audit.py`：检查导航点坐标、朝向和间距；
- `waypoint_recorder.py`：从标准 RViz 输入记录导航点；
- `generate_map_registered_tubes.py`：生成与地图坐标绑定的 Tube 路径。

比赛多阶段任务由 [`CompetitionSupervisor`](../src/competition_supervisor.cpp) 管理。
个人数据和录包放在仓库外部，通过命令行参数显式传入。
