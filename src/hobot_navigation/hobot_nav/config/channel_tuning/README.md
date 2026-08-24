# Channel tuning configuration

本目录保存 Tube 和 RPP 离线调参所需的参数：

- `channel_shadow_vehicle.yaml`：带转向、速度响应延迟和限幅的虚拟 Ackermann 车模；
- `channel_runtime_tuning.yaml`：运行时速度、前视距离和切换余量；
- `channel_cone_model.yaml`：锥桶几何与安全半径；
- `channel_sweep.yaml`：离线参数扫描范围。

这些配置用于几何生成、虚拟车模评估和实体车前的参数筛选。离线通过不等于真实车辆可以直接高速运行，最终参数仍需在实际车辆上做低速到目标速度的复核。
