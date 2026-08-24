# 导航工具中文说明

`tools/` 保存导航系统的离线调参、路径检查和数据分析工具。

| 类别 | 入口 | 作用 |
| --- | --- | --- |
| Ackermann 虚拟车模 | `ackermann_shadow_plant.py` | 模拟转向、加速度和命令延迟 |
| EKF/雷达分析 | `analyze_ekf_bags.py`、`analyze_lidar_landmarks.py` | 分析用户提供的数据和雷达几何 |
| Tube/RPP 调参 | `channel_tuning/` | 生成、检查和评估通道路径 |
| 倒车 LUT | `generate_reverse_gate_lut.py`、`validate_reverse_gate_paths.py` | 生成和验证 Ackermann 可行候选 |
| Native shadow replay | `nav2_native_shadow_replay.py`、`offline_mppi_shadow_sim.py` | 在不驱动车辆的情况下回放导航 |
| 速度命令分析 | `analyze_command_envelope.py`、`plot_limiter_ab.py` | 分析速度、曲率、加速度和限制器行为 |
| 底盘标定 | `stm32_ackermann_calibration.py` | 检查舵机和轮速标定 |
| 舵机/IMU 标定 | `calibration/` | PWM—转角拟合、六面 IMU、陀螺零偏和串口采集 |

推荐流程：

```text
真实车辆响应数据
        -> Ackermann 虚拟车模
        -> Tube/LUT 几何和净空检查
        -> 关闭电机输出的 Native Nav2 replay
        -> 低速实体车验证
```

需要 rosbag 的工具都通过参数显式接收数据路径。个人 rosbag、日志和结果目录不放在仓库中。

```bash
python3 tools/ackermann_shadow_plant.py --help
python3 tools/channel_tuning/generate_channel_tubes_v2.py --help
python3 tools/generate_reverse_gate_lut.py --help
python3 tools/nav2_native_shadow_replay.py --help
python3 tools/calibration/calibrate_imu_six_face.py --help
python3 tools/calibration/fit_servo_pwm_angle_pchip.py --help
```

标定工具的详细流程见 [`tools/calibration/README.md`](calibration/README.md)，英文说明见
[`tools/calibration/README_EN.md`](calibration/README_EN.md)。
