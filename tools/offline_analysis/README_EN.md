# Offline analysis tools

These scripts read user-provided rosbags, CSV files, or run results. They do not start the competition mission or connect to the real chassis.

| File | Purpose |
| --- | --- |
| `analyze_ekf_bags.py` | inspect wheel, IMU, and EKF odometry |
| `analyze_lidar_landmarks.py` | inspect LiDAR geometry and landmarks |
| `analyze_command_envelope.py` | inspect command, acceleration, and limiter behavior |
| `plot_limiter_ab.py` | compare limiter experiments |
| `plot_native_curvature_compare.py` | compare path curvature, vehicle curvature, and lateral error |

All data paths are passed explicitly on the command line. Personal rosbags, logs, and generated results stay outside the repository.
