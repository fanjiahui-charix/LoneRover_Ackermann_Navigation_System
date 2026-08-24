# origincarpro_base

这是 Ackermann 小车的下位机接口包，代码负责解析串口协议、发布原始轮式里程计和
IMU、发布 IMU/雷达静态 TF，并把经过上层限制的 `Twist` 命令转换成底盘协议。

公开导航链路使用的默认接口：

| 接口 | 默认值 |
| --- | --- |
| 原始轮式里程计 | `/odom/data_raw` |
| 原始 IMU | `/imu/data_raw` |
| 预处理 IMU | `/imu/fused/data_raw` |
| 速度命令 | `hobot_nav/navigation_core.launch.py` 中的 `/cmd_vel_safe` |
| 动态 TF | `ekf_fusion` 独占 `odom -> base_link` |
| 静态 TF | `base_link -> imu_link`、`base_link -> laser_link` |

下位机需要提供经过标定的轮速反馈、轴向和时间戳一致的 IMU、超时停车，以及能够
区分速度命令和反馈帧的串口协议。换板子、换 IMU 或调整舵机后，轮速比例、舵机 PWM
和转角限幅都要重新确认。

`config/origincarpro_base.yaml` 是当前车模参数。公开包不包含相机、图像处理或厂商
SDK；换车时至少要修改串口、底盘标定、IMU 标定和雷达外参。

标定脚本在仓库根目录的 [`tools/calibration/`](../../../tools/calibration/README.md)：
舵机 PWM—转角 PCHIP、六面 IMU 椭球拟合、静止陀螺零偏和下位机串口采集都在这里。
