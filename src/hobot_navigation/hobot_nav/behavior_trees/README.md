# behavior_trees

这里放 `hobot_nav` 自己的 Nav2 BT XML。

当前已经有一版更适合阿克曼底盘的恢复树：

- `navigate_to_pose_ackermann_recovery.xml`
- `navigate_through_poses_ackermann_recovery.xml`

这两份树保留了：

- 重规划
- 清空局部 / 全局代价地图
- 等待
- 小距离倒车

同时去掉了 Nav2 官方默认树里的 `Spin` 恢复动作，避免阿克曼底盘在恢复阶段尝试原地转向。

任务触发、路线选择和外部任务状态机不属于这个公开包。这里的 BT 只负责
一次 Nav2 导航 action 内部的规划、跟踪、重规划和恢复。
