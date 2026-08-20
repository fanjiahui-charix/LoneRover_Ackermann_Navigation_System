# LoneRover Ackermann Navigation System · ROS 2 竞赛导航系统

一套在真实比赛场地完成验证的 Ackermann 小车导航系统：轮式里程计与 IMU 的 EKF 融合惯导、1 cm 静态地图、N10 雷达锥桶感知、Nav2、完整任务状态机、倒车 LUT、通道 Tube/RPP，以及底盘速度控制和安全输出。稳定可以跑40s，再调一下速度和任务流程，能进30s。该仓库为我在2026地瓜机器人智慧医疗智能车创意组比赛中独自完成的核心源码。


[English README](README_EN.md)

仓库入口：

- [工具总览](tools/README_CN.md)：Ackermann 车模、EKF、倒车 LUT、Tube/RPP 和 Native X5 shadow；
- [比赛导航包](src/hobot_navigation/hobot_nav/README_CN.md)：启动、参数、地图、行为树和任务状态机；
- [比赛入口说明](src/hobot_navigation/hobot_nav/README_COMPETITION.md)：唯一的正式比赛启动命令；
- [通道调参文档](docs/channel_tuning/README.md)：Tube、RPP、连接段和候选部署流程。

---

## 中文

### 一、导航系统的构建

整套运行链路从底盘反馈开始，经过 EKF 定位、静态地图与动态障碍物、`SmacPlannerHybrid` 全局路径、MPPI/RPP 局部跟踪、速度平滑和车辆约束，最后把安全命令交给底盘。比赛任务的阶段推进由 `competition_supervisor` 统一管理。

#### 1. 定位：轮式里程计 + IMU EKF 融合惯导

下位机先通过串口把轮速反馈和 IMU 数据送上来。`origincarpro_base` 负责协议解析、轮速标定、IMU 零偏处理、静态 TF 和原始话题；`ekf_fusion` 再把轮式运动约束和 IMU 角速度观测融合成 `/odom`。

从融合结构看，这是一套松耦合的轮速—IMU EKF 组合惯导。底盘节点先分别发布 `/odom/data_raw` 和 `/imu/data_raw`，EKF 再将轮速观测与 IMU `gyro.z` 作为独立测量送入统一状态估计。紧耦合方案通常会把 IMU 原始数据、轮速模型和传感器残差统一写入观测模型；本系统采用分层预处理和独立观测输入，标定、验证和维护成本更低。IMU 不承担单独积分定位，动态 `odom -> base_link` 只由 EKF 发布，其他节点只读取这条 TF。

#### 2. 全局规划：静态地图上的可行路径

全局规划器读取全局 costmap，结合车辆 footprint、最小转弯半径和 Ackermann 运动学约束，从当前位姿搜索到目标位姿。项目使用 Nav2 Smac Planner 的 `SmacPlannerHybrid` 插件，即 Smac Planner 的 Hybrid-A* 模式。倒车入口 LUT 的候选验证也调用车上实际使用的同一套配置。

#### 3. 局部规划：实时避障和路径跟踪

MPPI 使用当前局部 costmap 和全局路径，在开放区域对速度、转向和障碍物进行短时域优化。它适合大厅这种空间相对开放、障碍物可能变化的区域。

进入窄通道后，MPPI 退出这段路径的临场搜索。通道使用提前生成的三组 Tube 轨迹，RPP 负责跟踪，Supervisor 根据锥桶侧别、车辆进度和任务状态切换轨迹。Tube 是轨迹资产，RPP 是局部跟踪器。

#### 4. Supervisor 与 Nav2 行为树

比赛任务的阶段状态写在 `competition_supervisor` 里：P 点、二维码、方向、扫码点、倒车入口、通道、出口和回 P 都由它推进。Nav2 仍然有 `BtNavigator` 和对应 XML，但它管理的是一段导航 action 内部的 ComputePath、FollowPath、重规划、取消和恢复。

两层职责不同：Supervisor 决定当前任务阶段，Nav2 BT 执行 Supervisor 发下去的导航动作。这样二维码、倒车和通道交界处的状态交接都集中在 Supervisor 里，BT 不负责替代比赛任务状态机。

#### 5. 控制器、速度控制、平滑和安全输出

规划器给出的是运动意图，底盘真正接收的是经过约束后的速度命令。这一层负责把“路径跟踪结果”变成车辆能够执行、变化不会太突兀、超时会停车的输出。当前命令链路可以概括为：

```text
ControllerServer /cmd_vel_nav_raw
              │
              ▼
Nav2 VelocitySmoother
  20 Hz 时间平滑、最大速度、加速度/减速度、超时处理
              │ /cmd_vel_nav
              ▼
competition_command_limiter
  前进/倒车限速、最小转弯半径、横向加速度、任务闸门
              │ /cmd_vel_safe
              ▼
底盘串口与舵机/电机执行
```

速度侧的自写逻辑还包括 `mycar_navigation` 里的曲率限速、刹车距离、目标点接近速度和加速度斜坡，以及 `adaptive_speed_limiter` 里的局部 costmap、雷达、路径、odom 和 TF 联合限速。它会根据曲率、前方可用距离、障碍物余量、距离目标点的距离和数据新鲜度压低速度；上下文过期时进入 fail-closed，停止输出危险速度。

速度平滑要解决的是时间连续性：控制器的速度变化不能直接跳到车上，必须受到最大速度、最大加速度、最大减速度和控制周期约束。最终 limiter 再检查 Ackermann 最小转弯半径、前进/倒车方向、横向加速度和任务阶段闸门。这样 Nav2、速度控制、底盘标定和安全策略各自有明确边界。

#### 6. 地图和障碍物：全局/局部 costmap 的分层

静态地图统一任务点、路径和障碍物的坐标。运行时的障碍物通过 costmap layer 叠加进去：

- **Static Layer**：加载 1 cm 静态地图，提供赛道边界和固定障碍；
- **Cone Layer**：雷达滤波、锥桶聚类后，把 `/cones/points` 转成带寿命的障碍；
- **Keepout/禁区 Layer**：写入不可进入的区域，防止规划器走进赛道禁区；
- **Inflation Layer**：根据车辆 footprint 和安全半径扩大障碍影响范围。

全局 costmap 主要给 `SmacPlannerHybrid` 计算全局路径，局部 costmap 给 MPPI、RPP 和安全链路做近距离判断。雷达在 RViz 里有 Marker，不代表规划器已经看到障碍；必须追到对应的 costmap 数据和规划结果。

地图分辨率也做过取舍。我试过 5 mm，甚至试过 1 mm。分辨率继续变细以后，costmap 的尺寸、内存和更新量明显增加，RDK X5 上的规划和调试都会变重；而车体 footprint、舵机误差、定位误差和锥桶检测误差，已经大于单纯把栅格从 1 cm 细化到 1 mm 带来的收益。最后保留 1 cm 静态地图，给规划留足障碍膨胀和车辆余量。

### 系统架构与数据流

```text
STM32 / 舵机 / 编码器 / IMU
          │ 串口协议、底盘反馈
          ▼
origincarpro_base ── 原始轮速、IMU、底盘静态 TF
          │
          ▼
ekf_fusion ───────── 轮式里程计 + IMU → odom → base_link
          │
          ├── 1 cm 静态地图 → Nav2 全局/局部代价地图
          ├── N10 雷达 → 滤波 → 锥桶聚类 → 锥桶层/禁区层
          ├── USB 相机 → 二维码 → 方向和任务分支
          └── USB 相机 → 立牌检测 → crop → 可选 VLM 描述
                                      │
                                      ▼
                    competition_supervisor / 完整任务状态机
                                      │
          ├── Nav2 Behavior Tree：导航动作、重规划、恢复
          ├── SmacPlannerHybrid：Nav2 Hybrid-A* 全局规划
          ├── MPPI：大厅等开放区域的局部规划
          └── Tube + RPP：窄通道的固定轨迹跟踪
                                      │
             VelocitySmoother → 自写限速/控制 → 安全输出 → 底盘
```

启动后，`origincarpro_base` 发布底盘反馈和 IMU，`ekf_fusion` 完成松耦合轮速—IMU 融合，并且只发布动态的 `odom -> base_link`。地图坐标系通过已有静态地图和固定起点位姿建立；当前现场命令同时启动 BEV stack，BEV 的计算和 `map -> odom` 修正由 Supervisor 按任务阶段控制。

### 二、系统特性

#### 定位与底盘

- **松耦合轮速—IMU EKF**：轮速观测约束车辆平移，IMU `gyro.z` 约束平面角速度；两类观测分别进入 EKF，最终只由 EKF 发布动态 `odom -> base_link`；
- **下位机反馈预处理**：包含串口协议、舵机 PWM 标定、轮速比例、角速度比例、底盘死区、IMU 零偏和静止伪角速度处理；
- **单一 TF 所有者**：避免底盘节点、定位节点和临时调试节点同时发布动态 odom TF；
- **固定起点定位模式**：比赛使用静态地图 + 起步位姿建立地图侧关系。

#### 地图与障碍物

- **1 cm 静态地图**：扫码点、倒车入口、Tube 轨迹、通道出口和 P 点在同一坐标系中定义；
- **N10 雷达处理链**：`/scan_raw -> /scan -> 锥桶聚类 -> /cones/points`，并保留可视化和诊断话题；
- **全局/局部 costmap 分层**：Static Layer、Cone Layer、Keepout/禁区 Layer、Inflation Layer 分工叠加；
- **障碍物进入规划器**：锥桶结果会写入全局和局部 costmap，Marker 只负责显示，真正影响 `SmacPlannerHybrid`、MPPI 和安全判断；
- **时间有效性控制**：锥桶有 TTL，过期障碍会清理；代价地图和 odom 的时间戳参与启动 readiness 检查。

#### 规划与控制

- **SmacPlannerHybrid（Hybrid-A* 模式）全局规划**：普通区域、二维码后的连接、倒车入口连接和回 P 都使用同一个全局规划器；离线 LUT 也用它检查 Ackermann 曲率、入口方向和车辆 footprint 可行性；
- **MPPI 局部规划**：使用局部 costmap 在大厅等开放区域实时优化局部控制；
- **Tube + RPP 窄通道方案**：离线生成三组不同贴边轨迹，运行时根据状态选择，RPP 负责跟踪；
- **速度控制链路**：`VelocitySmoother` 负责时间平滑，自写限速逻辑处理曲率、刹车距离、障碍余量和目标接近速度，最终 limiter 处理车辆约束并输出 `/cmd_vel_safe`；
- **路径和规划器分离**：Path 是规划结果或轨迹资产，Planner 负责生成/选择路径，Controller 负责跟踪路径，三者不混在一起。

#### 任务与工程工具

- **完整任务状态机 + Nav2 Behavior Tree**：任务状态管理 P 点、二维码、方向、扫码点、倒车入口、通道、出口和回 P，行为树管理每段导航动作；
- **有限状态和失败出口**：导航取消、超时、重规划失败、传感器过期和任务完成都有明确状态；
- **离线 LUT**：把倒车入口的连续搜索提前做掉，运行时只查有限候选并做 costmap 复核；
- **虚实结合**：保留 URDF/Xacro、RViz 车模、Ackermann shadow plant、Nav2 replay 和 rosbag 分析工具；
- **可观测性**：记录任务事件、路径、速度、odom、costmap、锥桶和运行状态，方便离线定位问题。

### 三、核心实现与关键取舍

#### 轮式里程计和 IMU 的 EKF

这部分可以准确地称为“松耦合轮速—IMU EKF 组合惯导”。底盘节点先完成串口解析、轮速和 IMU 预处理，再分别发布 `/odom/data_raw` 与 `/imu/data_raw`；`ekf_fusion` 将轮速的平面速度观测和 IMU `gyro.z` 观测送入 EKF，持续估计车辆的平面位姿和速度。它属于组合惯导/轮式惯性里程计，IMU 不承担独立积分定位。在比赛场地的完整运行测试中，累计位置误差达到厘米量级。

- 下位机负责串口协议、舵机和底盘反馈，发布原始轮速/里程计和 IMU。
- EKF 用轮式运动约束平移，用 IMU `gyro.z` 约束角速度和航向变化。
- 动态 `odom -> base_link` 只由 EKF 发布，避免多个节点抢 TF。
- 启动时处理 IMU 零偏、停车时的伪角速度和底盘死区。

正式运行的连续里程计由轮速和 IMU EKF 提供。BEV stack 由现场命令显式启动，但它的推理和 `map -> odom` 修正不是由普通定位节点直接接管，而是由比赛 Supervisor 按任务阶段打开和管理。

#### 倒车 LUT

LUT 的价值在于把车上最容易发散的一段搜索提前做完。LUT 由离线程序批量生成：先遍历大厅中可能的入口位姿，再放到 RDK X5 上调用比赛实际使用的 `SmacPlannerHybrid`，逐个位姿规划到入口，检查路径是否成功、是否满足 Ackermann 曲率、车辆 footprint 是否碰障碍。通过实际车端配置筛掉失败候选以后，再把可行入口、路径长度、方向和对应的二维码区域整理成 LUT。比赛运行时冻结扫码位姿，查找附近的有限候选，再用当前 costmap 做一次复核。这样 LUT 和车上的真实规划器保持一致，运行时也不用重新遍历整片大厅。

#### Tube + RPP

通道运行时使用三条贴不同边的 Tube 轨迹，Supervisor 根据锥桶侧别、车辆进度和任务状态选择轨迹，RPP 负责连续跟踪。Tube 是固定赛道上的轨迹资产，不是运行时重新建图或重新搜索出来的路径。轨迹可以在虚拟车模中结合车辆延迟、转向限幅和障碍余量离线检查，再将通过检查的参数用于车端运行。

### 四、系统要求

#### 主机和车端

- 主机：Ubuntu 22.04，负责源码检查、Python 测试和 X5 交叉编译；
- 车端：RDK X5 aarch64，Ubuntu 22.04、ROS 2 Humble/TROS 和对应硬件运行时；
- 构建：工作区同级准备 X5 sysroot，默认位置是 `../sysroot_docker/usr_x5`；
- 通信：车端传感器、底盘和相机必须在同一个 ROS Domain 内，时间戳要连续；

#### 对下位机的要求

下位机除了接收速度命令，还需要向上位机提供完整反馈。导航链路至少需要：

- 可读的轮式里程计/轮速反馈，方向和单位必须经过标定；
- IMU 的角速度和线加速度，坐标系、轴方向、零偏和采样时间正确；
- 舵机 PWM 与实际转角的对应关系，以及左右转角限幅；
- 串口协议能够区分速度命令、轮速反馈、IMU 数据和错误状态；
- 底盘能够在上层超时、急停或安全节点失联时停止；
- 车体、IMU、雷达之间的静态 TF 与真实安装位置一致。

默认配置里可以看到串口、话题和标定参数，但换板子、换 IMU、换舵机或改安装位置后，不能直接沿用旧参数。先做底盘原始数据检查，再启动 EKF 和导航。

#### 对雷达、地图和相机的要求

- N10 雷达需要稳定发布 `/scan_raw`，`scan_time` 和 `time_increment` 不能是无效值；
- 静态地图、P 点、二维码区域、倒车入口和 Tube 轨迹必须使用同一个地图坐标系；
- 相机和二维码节点需要提供可锁存的合法结果，扫码失败必须有超时和备用状态；
- VLM、立牌识别和 HDMI 属于可选旁路，不能成为基础导航启动的必要条件。

### 五、关键话题与 TF

下面列的是最终导航链路中最重要的接口，具体名称可以通过各包的 YAML 和 launch 参数覆盖。

| 方向 | 话题/接口 | 作用 |
| --- | --- | --- |
| 输入 | `/odom/data_raw` | 下位机原始轮式反馈 |
| 输入 | `/imu/data_raw`、`/imu/fused/data_raw` | 原始/预处理 IMU |
| 输出 | `/odom` | EKF 融合后的导航里程计 |
| 输入 | `/scan_raw` | N10 原始雷达 |
| 输出 | `/scan` | 滤波后的雷达数据 |
| 输出 | `/cones/points`、`/cones/poses` | 锥桶点和聚类结果 |
| 输入 | `/map` | 1 cm 静态地图 |
| 输入 | `/code` | 二维码结果 |
| 输出 | `/race/direction` | 顺/逆时针方向 |
| 输出 | `/race/mission_event`、`/race/mission_audit` | 任务状态和审计信息 |
| 输出 | `/cmd_vel_safe` | 通过速度控制与安全限制后的底盘命令 |

| TF | 发布/用途 |
| --- | --- |
| `odom -> base_link` | 由 `ekf_fusion` 动态发布，不能再由其他定位节点抢占 |
| `map -> odom` | 由固定起点和地图侧关系建立；BEV 修正由 Supervisor 按阶段控制 |
| `base_link -> imu_link` | IMU 静态外参 |
| `base_link -> laser` | N10 雷达静态外参 |

启动后建议先用 `ros2 topic info`、`ros2 topic hz`、`ros2 topic echo` 和 TF 检查确认话题所有者、频率、时间戳和坐标系，再启动任务。

### 六、目录结构

```text
src/
├── mycar/
│   ├── origincarpro_base/       # STM32 串口、底盘反馈、轮速、IMU、静态 TF
│   ├── ekf_fusion/              # 轮式里程计 + IMU 的 2D EKF
│   ├── origincar_description/   # 车模、URDF/Xacro、RViz 离线可视化
│   ├── mycar_navigation/        # 普通导航接口
│   ├── mycar_calibration/       # 相机和车辆标定工具
│   ├── mycar_sensor/             # USB 相机
│   └── mycar_record/             # rosbag 录制、运行状态快照和离线复盘辅助
├── hobot_navigation/
│   ├── hobot_nav/               # 比赛入口、Supervisor、状态机、地图、LUT、Tube
│   ├── n10_driver/              # N10 雷达驱动
│   ├── lidar_perception/        # 滤波、锥桶聚类和 costmap layer
│   └── adaptive_speed_limiter/  # 速度限制器
├── hobot_vision/
│   ├── hbmem_wechatcode/        # 二维码识别
│   ├── board_crop/              # 立牌 crop 和检测适配
│   └── hobot_ollama/            # 异步 VLM sidecar
├── hobot_msgs/                  # 自定义 ROS 消息
└── 3rdparty/                    # Hobot/TROS 接口包
robot_dev_config/                # X5 交叉编译和依赖配置
tools/                           # LUT、Tube、EKF 和离线分析工具
docs/                            # Tube、RPP、部署和调参记录
findings.md                      # 工程附录（历史整理，不参与运行）
progress.md                      # 阶段附录（历史整理，不参与运行）
task_plan.md                     # 复现与调参计划（不参与运行）
x5_build.sh                      # X5 构建入口
scripts/deploy_x5.sh             # 通用部署脚本
```

### 七、构建与运行

#### 构建

目标是 Ubuntu 22.04、ROS 2 Humble/TROS 和 RDK X5 aarch64。X5 交叉编译需要与工作区同级的 sysroot：

```text
/root/
├── ros_workspace/
└── sysroot_docker/usr_x5/
```

```bash
source /opt/ros/humble/setup.bash
./x5_build.sh --up-to hobot_nav
```

主机主要负责交叉编译和静态检查，不能把主机上的 x86 环境当成车端运行环境。第一次打通时建议使用 `--up-to hobot_nav`，已有依赖产物以后再选择目标包增量构建。

#### 正式比赛运行

```bash
export X5_DEPLOY_HOST='root@YOUR_VEHICLE_IP'
./scripts/deploy_x5.sh
```

车上先关闭桌面显示，再确认急停、串口设备、P 点位姿和 ROS Domain：

```bash
systemctl stop lightdm

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
`channel_v2_admission_sha256` 在公开文档中使用占位符，车端使用现场配置值。
ROS Domain 231 用于隔离不同车辆的 DDS 数据。

### 八、赛题具体怎么做

#### 任务一：P 点到二维码区域

车辆从 P 点出发，通过 `SmacPlannerHybrid` 生成全局路径，MPPI 负责开放区域的局部跟踪，进入预设二维码区域后完成识别。二维码节点持续运行并锁存第一个合法结果；结果的奇偶性决定顺时针或逆时针任务分支，主扫码点和备用扫码点由 Supervisor 统一管理。

大厅中的锥桶可能遮挡远处信息，因此系统在接近扫码区域时保留有限的中途重规划和目标点切换。全局规划负责路线连通性，MPPI 负责当前局部障碍和速度控制，两者共同完成从 P 点到扫码区域的过渡。


#### 任务二：二维码之后倒车进入窄通道

得到二维码结果后，先冻结扫码时的车辆位姿，再通过倒车 LUT 查询入口候选。LUT 由离线程序遍历大厅入口区域生成，候选包括直线倒车、不同半径的右后方倒车等几何运动片段。每个候选都会检查 Ackermann 曲率、车身扫掠范围、障碍距离和与入口的连接关系；锥桶信息可以提前筛除明显不可行的搜索分支。最终候选在 RDK X5 上调用实际使用的 `SmacPlannerHybrid` 逐个位姿验证，并由当前全局/局部 costmap 做运行时复核。

完成倒车后，车辆使用 `SmacPlannerHybrid` 连接到通道入口。通道内部由三组分别贴近不同侧边的 Tube 轨迹组成，Supervisor 根据锥桶侧别、车辆进度和任务状态选择轨迹，RPP 负责连续跟踪。雷达锥桶结果写入 Cone Layer 和禁区层，用于轨迹选择、局部规划和安全约束。导航点只负责阶段交接，不承担通道内的连续控制。

通道旁的立牌识别是可选旁路。YOLO 负责在限定窗口检测立牌，VLM 只处理质量较高的 crop，结果通过异步消息回传。识别失败时导航仍然可以继续，文字识别请求不参与底盘控制。

#### 任务三：通道出口返回 P 点

通道结束后，Supervisor 将控制权交回常规导航链路，使用当前 EKF 位姿重新规划回 P 点。`SmacPlannerHybrid` 负责全局路径，MPPI 负责开放区域局部跟踪，速度控制器和安全输出继续作为最后一层约束。任务完成还要结合动作结果、当前位置、停止状态和任务事件进行确认。

Tube 轨迹只覆盖通道内部的通过段。车辆出通道后，Supervisor 将控制权交回普通导航链路，使用当前 EKF 位姿重新规划回 P 点，避免把固定通道轨迹延伸到出口和开放区域。

### 九、导航调试经验和容易出问题的地方

#### 先区分 Path、Planner、Controller 和车辆

这几个东西经常被混在一起：

- **Planner**：根据地图、代价和运动学约束生成路径；
- **Path**：Planner 输出的位姿序列，或者离线保存的 Tube/LUT 轨迹；
- **Controller**：根据当前位姿和 Path 生成速度、转角或 `cmd_vel`；
- **Vehicle**：真实底盘执行命令，可能还有舵机延迟、死区和限幅。

车走偏时，不能只看最后画出来的轨迹。先看 Planner 给的 Path 是否绕开了障碍，再看车辆位姿是否正确，最后看 Controller 输出和底盘实际反馈。Path 已经穿过锥桶，调 RPP 没用；Path 是对的但车跟不上，才去查 lookahead、速度、转向限幅和舵机响应。

#### 控制导航点数量

通道里一开始放了很多导航点，想通过点位把车“掰”进去，结果 Nav2 action、行为树、重规划和控制器之间的交接变得非常复杂。点太密时，前一个目标还没稳定结束，后一个目标又来了，容易出现取消、重发、路径短到不可用、BT 恢复行为反复触发等问题。

最后通道改成少量状态切换 + Tube 轨迹，导航点只负责阶段交接，连续跟踪交给 RPP。固定赛道里，少一些导航点反而更容易调。

#### 车原地自转，先查定位和路径方向

自转不一定是控制器参数问题，常见原因有两类：

1. **定位问题**：`odom` 的 yaw 在车不动时变化，IMU 角速度有零偏，轮速方向反了，或者 `map -> odom` 在跳；
2. **路径/规划问题**：当前目标 yaw 不合理，Path 在车辆附近出现反向切线，Hybrid-A* 或倒车路径方向和控制器预期不一致。

判断方法是同时录 `/odom`、`/tf`、全局 Path、局部 Path、原始/安全速度和底盘反馈。如果 RViz/Foxglove 里的车体姿态自己在动，先处理定位；如果定位稳定但 Path 在车前突然翻方向，再查规划器和路径资产。

提高 goal yaw 或 xy 容忍度有时能解决“已经到了但 action 不结束”。容忍度应该服务于任务区域的停车要求，定位漂移仍然要回到 EKF 和 TF 去修。

#### 锥桶看到了，但规划器可能还没看到

要按这条链路排查：

```text
/scan_raw → /scan → cone_detector → /cones/points
          → TF → ConeLayer/KeepoutLayer
          → global/local costmap → Planner/Controller
```

如果只看到了 Marker，就还不能说明避障生效。重点检查坐标系、时间戳、QoS、锥桶 TTL、全局/局部 costmap 是否更新，以及规划器取样的地图分辨率。锥桶层闪烁、障碍过期太快、footprint 和锥桶半径没有留余量，都会出现“检测到了但车还是擦过去”的情况。

#### 用 Foxglove 看数据，RViz 做空间确认

车端直接运行 RViz 在 X5 上容易产生较高负载和显示延迟，尤其是全局 costmap、局部 costmap、雷达点、相机和多个 Path 同时打开时。我更建议：

- 用 `mycar_record` 录导航话题和运行快照；
- 用 Foxglove 查看话题频率、时间戳、Path、TF、速度和事件时间线；
- 用 RViz 做地图、TF、锥桶层和机器人空间关系的最后确认；
- 不要在车辆运动时为了看图打开一堆高带宽显示和调试节点。

很多问题来自车端负载、DDS、相机/雷达回调和可视化同时运行造成的实时性下降。先把数据录下来，再离线看，通常比盯着一台卡顿的 RViz 更快。

### 比赛资料

仓库还保留了我实际使用过的两份比赛参考资料，放在 [`docs/competition/`](docs/competition/)：

- [赛项方案（DOCX）](docs/competition/21届全国大学生智能汽车竞赛地瓜机器人赛项方案-20260316.docx)
- [比赛流程及裁判员参考手册（PDF）](docs/competition/FireShot%20Capture%20114%20-%20裁判手册%20-%20第二十一届全国大学生智能汽车竞赛地瓜机器人智慧医疗赛比赛流程及裁判员参考手册.pdf)

它们用于解释任务约束、评分流程和代码里的状态设计，不参与构建和运行。

仓库还保留了三份工程附录。它们用于保存背景和复现线索，不是当前运行说明，也不参与构建和启动：

- [工程附录：判断与阶段记录](findings.md)
- [工程附录：进度和路线记录](progress.md)
- [工程附录：复现与调参计划](task_plan.md)

### 虚实结合和离线调参

这部分我专门保留了，因为很多参数如果每次都上车试，调参效率会非常低。我的做法是把真实车的数据、RDK X5 上的实际 Nav2 配置和一个带延迟响应的 Ackermann 车模放到同一条离线验证链路里。

#### 1. 用真实数据拟合车模响应

`tools/offline_mppi_shadow_sim.py` 可以读取真实 rosbag，对 `/cmd_vel_safe` 和 `/odom` 做时间对齐，估计命令传输延迟、纵向响应时间常数和角速度响应时间常数。车模将命令延迟、速度变化和转向响应显式建模，避免把车辆当作瞬时响应系统，使离线轨迹与真实车辆更接近。

#### 2. Ackermann shadow plant

`tools/ackermann_shadow_plant.py` 根据速度、转角、加速度、车辆最小转弯半径和响应延迟推进虚拟车辆。Nav2、costmap、MPPI、RPP、速度平滑和 limiter 可以对着虚拟车辆运行，输出通过 shadow topic 隔离，不能直接碰真实车的 `/cmd_vel_safe`。

#### 3. 用真实 X5 Nav2 做最终复核

电脑上的轻量模型适合快速扫参数，最终候选还要放到 RDK X5 上用实际安装的 Nav2、`SmacPlannerHybrid`、MPPI、VelocitySmoother 和 costmap 做 native shadow replay。这样可以把主机上跑得通、车上跑不动的候选尽早筛掉，也能发现 X5 CPU、DDS 和传感器回调带来的实时性问题。

#### 4. LUT、Tube 和参数一起验证

倒车 LUT 在 X5 上调用实际的 `SmacPlannerHybrid` 逐个位姿生成和检查。Tube 则用真实车体尺寸、最小转弯半径、footprint 和锥桶避障约束生成，再用带延迟的虚拟车跑 RPP 跟踪。最后拿少量候选到真实车上做低速和完整任务验证。

- `src/mycar/origincar_description/`：车辆 URDF/Xacro、RViz 模型和显示启动文件；
- `tools/ackermann_shadow_plant.py`：根据速度、转角、加速度和舵机响应模拟车辆；
- `tools/nav2_native_shadow_replay.py`：真实 Nav2 节点 + 虚拟底盘的闭环回放；
- `tools/offline_mppi_shadow_sim.py`：直接读取 rosbag 做离线轨迹和参数对比；
- `tools/generate_reverse_gate_lut.py`、`tools/generate_channel_tubes_v2.py`：倒车 LUT 和 Tube 轨迹的离线生成工具。
