# 导航工具中文说明

`tools/` 保存导航系统的离线调参、路径检查和数据分析工具。

目录按职责整理：[`vehicle_model/`](vehicle_model/README.md) 放 Ackermann 车模和虚拟车辆回放，[`reverse_entry/`](reverse_entry/README.md) 放倒车入口 LUT，[`channel_tuning/`](channel_tuning/README.md) 放 Tube/RPP 资产，[`calibration/`](calibration/README.md) 放下位机标定，[`offline_analysis/`](offline_analysis/README.md) 放 rosbag 和运行结果分析。

| 类别 | 入口 | 作用 |
| --- | --- | --- |
| Ackermann 虚拟车模 | `vehicle_model/ackermann_vehicle_simulator.py` | 模拟转向、加速度和命令延迟 |
| EKF/雷达分析 | `offline_analysis/analyze_ekf_bags.py`、`offline_analysis/analyze_lidar_landmarks.py` | 分析用户提供的数据和雷达几何 |
| Tube/RPP 调参 | `channel_tuning/` | 生成、检查和评估通道路径 |
| 倒车入口 LUT | `reverse_entry/` | 生成、清理、采样和验证 Ackermann 可行候选 |
| 虚拟车辆回放 | `vehicle_model/nav2_virtual_vehicle_replay.py`、`vehicle_model/offline_vehicle_response_sim.py` | 在不驱动车辆的情况下回放导航 |
| 速度命令分析 | `offline_analysis/analyze_command_envelope.py`、`offline_analysis/plot_limiter_ab.py` | 分析速度、曲率、加速度和限制器行为 |
| Ackermann 车模 | `vehicle_model/ackermann_vehicle_model.py` | 提供几何换算、舵机查表和响应检查 |
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
python3 tools/vehicle_model/test_ackermann_vehicle_simulator.py
python3 tools/channel_tuning/generate_tube_paths.py --help
python3 tools/reverse_entry/generate_reverse_entry_lut.py --help
python3 tools/vehicle_model/nav2_virtual_vehicle_replay.py --help
python3 tools/calibration/calibrate_imu_six_face.py --help
python3 tools/calibration/fit_servo_pwm_angle_pchip.py --help
```

标定工具的详细流程见 [`tools/calibration/README.md`](calibration/README.md)，英文说明见
[`tools/calibration/README_EN.md`](calibration/README_EN.md)。
