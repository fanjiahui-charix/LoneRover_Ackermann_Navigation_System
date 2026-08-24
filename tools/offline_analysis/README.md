# 离线分析工具

这里的脚本只读取用户自行准备的 rosbag、CSV 或运行结果，不启动比赛任务，也不连接真实底盘。

| 文件 | 作用 |
| --- | --- |
| `analyze_ekf_bags.py` | 分析轮速、IMU 和 EKF 里程计 |
| `analyze_lidar_landmarks.py` | 分析雷达几何和地标数据 |
| `analyze_command_envelope.py` | 分析速度命令、加速度和限制器行为 |
| `plot_limiter_ab.py` | 对比限制器实验结果 |
| `plot_native_curvature_compare.py` | 对比路径曲率、车辆曲率和横向误差 |

数据路径都通过命令行参数传入，个人 rosbag、日志和生成结果不放在仓库里。
