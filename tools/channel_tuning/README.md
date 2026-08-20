# Tube and RPP tuning tools

本目录保留不依赖任务层、视觉和设备驱动的几何与离线调参工具。

- `generate_channel_tubes_v2.py`：生成中心、内侧、外侧 Tube 候选；
- `evaluate_channel_tubes.py`：检查路径顺序、闭合、footprint 和曲率；
- `generate_channel_connectors_v2.py`：生成 Tube 之间的连接段；
- `evaluate_channel_connectors.py`：检查连接段端点和净空；
- `generate_channel_cone_avoid_map.py`：生成锥桶侧向避障表；
- `evaluate_channel_avoid_map.py`：评估避障表；
- `materialize_channel_avoidance_scenario.py`：生成离线避障场景；
- `evaluate_channel_rpp_run.py`：评估用户提供的 RPP 运行结果；
- `freeze_channel_runtime_tuning.py`：从筛选结果生成运行参数；
- `visualize_channel_assets.py`：绘制路径、曲率和 footprint 检查图。

这些工具的输出目录应放在仓库外。几何通过只代表离线约束通过，实际车辆仍需做低速验证。
