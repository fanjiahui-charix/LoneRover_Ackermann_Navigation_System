# OriginCar vehicle description

这是比赛小车的可视化模型，给 RViz、机器人状态发布器和离线调参使用。
它不是比赛运行时的底盘驱动，也不会向真实车辆发送任何控制命令。

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch origincar_description display.launch.py
```

真实 Nav2 与虚拟车辆闭环调参见仓库根目录的 `tools/`：

- `tools/vehicle_model/ackermann_vehicle_simulator.py`：带速度和舵机一阶响应的 Ackermann 虚拟车模；
- `tools/vehicle_model/nav2_virtual_vehicle_replay.py`：让真实 Nav2 节点接收虚拟底盘反馈，命令全部走 virtual topic；
- `tools/vehicle_model/offline_vehicle_response_sim.py`：使用 rosbag 做不启动 ROS 节点的离线回放和参数对比。

模型尺寸来自比赛车辆标定参数。网格文件仅用于可视化，运行导航和离线回放不依赖它们。
