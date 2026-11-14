# ekf_fusion

这是轮式里程计与 IMU 的二维 EKF 融合包，负责把下位机的原始反馈整理成导航使用的
`/odom`，并发布唯一的动态 `odom -> base_link`。它不是单独的惯导积分器：轮速提供
平面位移约束，IMU 提供角速度和姿态相关观测，滤波器在统一状态空间中完成预测、更新
和异常观测抑制。

## 主要接口

- 输入：`/odom/data_raw`、`/imu/data_raw` 或 `/imu/fused/data_raw`；
- 可选输入：`/lidar_odom`，由外部或本仓库的 `simple_lidar_odom` 提供；
- 输出：`/odom` 以及对应的动态 TF；
- 服务：`SetPose`、`ToggleFilterProcessing`。

配置位于 `config/ekf.yaml` 和 `config/ekf_with_lidar_odom.yaml`。传感器的坐标系、时间戳、
协方差和静态 TF 必须先正确，EKF 参数不能替代底盘标定。

```bash
ros2 launch ekf_fusion ekf_fusion_only.launch.py \
  ekf_params_file:=src/mycar/ekf_fusion/config/ekf.yaml \
  cleanup_existing:=false
```

这是独立调试入口，默认同时启动底盘、地图预览、RViz 和单位 `map -> odom`。
不要与另一套正在运行的底盘/EKF/TF 重复启动。示例关闭了自动清理旧节点；参数文件
路径相对于当前工作区。`maps/` 中保留的 1 cm 地图与 `hobot_nav/maps/` 内容相同，
用于本包独立调试，避免要求 EKF 包依赖导航集成包。

完整导航链路使用：

```bash
ros2 launch hobot_nav navigation_core.launch.py
```

公开包不包含雷达串口驱动、相机、任务识别或建图入口；雷达里程计如果要接入，必须
明确设置 `odom1` 的话题、时间戳和协方差，并确认它不会抢占 EKF 的 TF。

底盘轮速与 IMU 的标定、零偏和时间戳检查应先于融合参数调节；增加观测源不能替代
这些检查。公开参数的 EKF 目标输出频率为 50 Hz，实际频率需在目标设备上测量。
