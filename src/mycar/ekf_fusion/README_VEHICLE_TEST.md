# Vehicle test checklist

上车前确认：

- `/odom/data_raw`、`/imu/data_raw` 的频率和时间戳连续；
- 静止时 yaw 速率没有持续偏置，直线行驶时轮速符号正确；
- `base_link -> imu_link` 与 `base_link -> laser_link` 外参真实有效；
- 只有 `ekf_fusion` 发布 `odom -> base_link`；
- 低速直线、左右转和停车后，`/odom` 没有明显跳变；
- 速度命令经过平滑和 Ackermann 限制器后再进入底盘。

测试数据和 rosbag 请放在仓库外部，不要提交进公开仓库。
