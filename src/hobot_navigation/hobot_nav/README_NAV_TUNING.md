# Navigation tuning and virtual vehicle

公开调参链路保留了 Tube、倒车 LUT、RPP、速度约束和延迟 Ackermann 车模。推荐流程是：

1. 固定地图分辨率、车辆 footprint 和最小转弯半径；
2. 生成或选择路径候选；
3. 在虚拟车模中加入转向、速度响应延迟和限幅；
4. 检查曲率、净空、横向误差、速度和命令连续性；
5. 在真实车上从低速开始复核。

1 cm 是当前公开地图配置。5 mm 和 1 mm 可用于几何诊断，但会增加实时计算和可视化负担。

```bash
python3 tools/vehicle_model/test_ackermann_vehicle_simulator.py
python3 tools/channel_tuning/evaluate_channel_tubes.py --help
python3 tools/vehicle_model/nav2_virtual_vehicle_replay.py --help
python3 tools/vehicle_model/offline_vehicle_response_sim.py --help
```

数据路径必须通过参数显式传入，个人录包和实验输出不属于仓库内容。
