# Tube and RPP offline tuning

公开的通道调参流程围绕四类资产：Tube 路径、连接段、锥桶避障几何和 RPP 参数。所有生成结果都应该写入仓库外的输出目录，输入地图、车辆 footprint 和参数文件必须显式指定。

推荐顺序：

1. 用 `tools/channel_tuning/generate_tube_paths.py` 生成候选 Tube；
2. 用 `evaluate_channel_tubes.py` 检查闭合、曲率、footprint 和地图净空；
3. 用 `generate_channel_connectors.py` 生成路径连接段，并用 `evaluate_channel_connectors.py` 复核；
4. 用 `generate_channel_cone_avoid_map.py` 和 `evaluate_channel_avoid_map.py` 检查锥桶侧向选择；
5. 用 `channel_shadow_vehicle.yaml` 和 `channel_runtime_tuning.yaml` 做虚拟车模延迟分析；
6. 用真实车辆做低速复核，再冻结运行参数。

几何检查通过只代表路径资产满足离线约束，不代表真实车辆可以直接高速运行。真实验证仍需检查定位误差、控制延迟、轮胎打滑和命令输出。
