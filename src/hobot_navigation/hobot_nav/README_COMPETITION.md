# 比赛入口说明

## 构建与车端依赖

任务源码位于 [`src/competition_supervisor.cpp`](src/competition_supervisor.cpp)，Action 交接由 [`src/motion_action_arbiter.cpp`](src/motion_action_arbiter.cpp) 统一管理。代码与启动链路恢复自 [TarochLee/ros_workspace@aa018d6](https://github.com/TarochLee/ros_workspace/tree/aa018d6627b272c0797bab58faaa3dd07884f2d1)。

```bash
colcon build --packages-up-to hobot_nav \
  --cmake-args -DBUILD_COMPETITION_SUPERVISOR=ON
source install/local_setup.bash
```

车端需安装 N10、相机、QR、VLM、HDMI 与 BEV 链路对应的 ROS 包，并准备相机标定、BEV 模型/语义地图；通道准入资产随包安装在 `runtime/channel_v2/`。包名及 X5 构建方法见 [DEPENDENCIES.md](../../../docs/DEPENDENCIES.md)。比赛命令使用包内通道资产和对应摘要；相机、BEV 与 VLM 参数使用实际部署值。

这是比赛现场使用的完整导航运行入口说明，记录第二十一届全国大学生智能汽车竞赛地瓜机器人智慧医疗赛的运行方式、地图、定位、规划、通道和速度输出链路。

## 比赛运行命令

比赛现场先关闭桌面显示：

```bash
systemctl stop lightdm
```

然后在车端执行完整比赛入口：

```bash
export ROS_DOMAIN_ID=231
CHANNEL_V2_DIR="$(ros2 pkg prefix hobot_nav)/share/hobot_nav/runtime/channel_v2"

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
  channel_v2_asset_directory:="$CHANNEL_V2_DIR" \
  channel_v2_admission_sha256:=f1b8d865214c67f7f4536806c66f5de47b5a055918a2a5f57fcfb9e251837870 \
  channel_runtime_tuning_file:="$CHANNEL_V2_DIR/configuration/channel_runtime_tuning.yaml" \
  readiness_timeout_sec:=90 \
  capture_diagnostics_enabled:=false \
  competition_quiet_mode:=false \
  competition_log_level:=info \
  log_level:=error
```

这条命令加载已有的 1 cm 静态地图，不启动建图。`vlm_server_ip` 使用现场服务器地址，通道摘要对应包内 `runtime/channel_v2/runtime_admission.csv`。
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
