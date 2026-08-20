# 比赛入口说明

这是比赛现场使用的完整导航运行入口说明，记录第二十一届全国大学生智能汽车竞赛地瓜机器人智慧医疗赛的运行方式、地图、定位、规划、通道和速度输出链路。

## 比赛运行命令

比赛现场先关闭桌面显示：

```bash
systemctl stop lightdm
```

然后在车端执行完整比赛入口：

```bash
export ROS_DOMAIN_ID=231

ros2 launch hobot_nav competition_runtime.launch.py \
  start_mission:=true \
  mission_scope:=full \
  start_camera:=true \
  use_qr:=true \
  use_vlm:=true \
  use_hdmi:=true \
  launch_foxglove_bridge:=false \
  vlm_server_ip:=YOUR_VLM_SERVER_IP \
  enable_cone_detector:=true \
  launch_bev_stack:=true \
  base_cmd_vel_topic:=/cmd_vel_safe \
  speed_profile:=race_07 \
  linear_speed_limit:=0.70 \
  allow_high_speed_navigation:=true \
  reverse_speed_limit:=0.35 \
  race_continue_enabled:=true \
  direct_reverse_to_channel_entry_enabled:=true \
  qr_midcourse_replan_x:=1.6 \
  channel_avoidance_v2_enabled:=true \
  channel_templates_runtime_allowed:=true \
  channel_v2_asset_directory:=/root/ros_workspace/channel_v2_active \
  channel_v2_admission_sha256:=YOUR_CHANNEL_V2_ADMISSION_SHA256 \
  channel_runtime_tuning_file:=/root/ros_workspace/channel_v2_active/configuration/channel_runtime_tuning.yaml \
  readiness_timeout_sec:=90 \
  capture_diagnostics_enabled:=false \
  competition_quiet_mode:=false \
  competition_log_level:=info \
  log_level:=error
```

这条命令加载已有的 1 cm 静态地图，不启动建图。`vlm_server_ip` 和
`channel_v2_admission_sha256` 在文档中使用占位符，车端使用现场配置值。
ROS Domain 231 用于隔离不同车辆的 DDS 数据。

## 运行链路

```text
下位机轮速反馈 + IMU
        ↓
origincarpro_base
        ↓
轮速里程计 + IMU EKF
        ↓
固定起点与 map/odom 关系
        ↓
已有 1 cm 静态地图
        ↓
Nav2 代价地图与 SmacPlannerHybrid
        ↓
开放区域 MPPI
        ↓
窄通道 Tube/RPP
        ↓
速度平滑与 Ackermann 限制
        ↓
/cmd_vel_safe
        ↓
底盘执行
```

## 比赛中的导航结构

- 定位使用轮速和 IMU 的松耦合 EKF；
- 全局路径使用 Nav2 `SmacPlannerHybrid`，它本身就是 Hybrid-A* 规划器；
- 开放区域使用 MPPI 进行局部规划和实时避障；
- 倒车入口通过真实 Ackermann 车模离线生成 LUT，在线查表并复核；
- 窄通道使用中心、内侧、外侧三组 Tube 路径，由 RPP 跟踪；
- 速度平滑、曲率限速、倒车限速和安全输出共同约束底盘命令；
- 通道结束后交回普通导航链路，使用当前 EKF 位姿规划回终点区域。

## 启动前检查

确认急停、起点位姿、ROS Domain、下位机反馈、IMU、雷达、静态 TF、costmap 更新和通道运行参数都正常。车辆在所有 readiness 检查通过前保持静止。

完整的导航包说明见 [README_CN.md](README_CN.md)，离线 LUT、Tube 和虚拟车模工具见 [tools/README_CN.md](../../../tools/README_CN.md)。
