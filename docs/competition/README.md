# 比赛任务链路

车辆通过 QR 任务选择、倒车入口、通道跟踪和返程完成比赛导航。任务状态机由 [CompetitionSupervisor](../../src/hobot_navigation/hobot_nav/src/competition_supervisor.cpp) 管理；[MotionActionArbiter](../../src/hobot_navigation/hobot_nav/src/motion_action_arbiter.cpp) 统一发送、取消和交接 Nav2 Action。

完整启动命令见[比赛入口说明](../../src/hobot_navigation/hobot_nav/README_COMPETITION.md)，安装依赖与构建选项见 [DEPENDENCIES.md](../DEPENDENCIES.md)。比赛流程、计时和罚则以主办方发布的赛项资料为准。

[English](README_EN.md)
