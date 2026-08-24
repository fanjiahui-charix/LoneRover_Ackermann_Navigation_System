# 虚拟车辆与 Ackermann 车模

这里放的是和真实车辆解耦的车模与回放工具。它们只发布虚拟车辆状态，不向真实底盘发送 `/cmd_vel_safe`。

| 文件 | 作用 |
| --- | --- |
| `ackermann_vehicle_model.py` | 车辆几何、舵机查表、转角与角速度换算 |
| `ackermann_vehicle_simulator.py` | 加入速度/转向响应、延迟和限幅的 Ackermann 虚拟车模 |
| `nav2_virtual_vehicle_replay.py` | 让真实 Nav2 节点接收虚拟底盘反馈，做闭环复核 |
| `offline_vehicle_response_sim.py` | 从用户提供的 rosbag 估计命令延迟和车辆响应 |
| `test_ackermann_vehicle_simulator.py` | 车模和几何换算的轻量测试 |

常用入口：

```bash
python3 tools/vehicle_model/test_ackermann_vehicle_simulator.py
python3 tools/vehicle_model/nav2_virtual_vehicle_replay.py --help
python3 tools/vehicle_model/offline_vehicle_response_sim.py --help
```

`nav2_virtual_vehicle_replay.py` 需要 ROS 2/Nav2 环境；`offline_vehicle_response_sim.py` 需要用户自行准备 rosbag。真实车参数修改后，先更新 `ackermann_vehicle_model.py` 和车模配置，再进行实体车低速验证。
