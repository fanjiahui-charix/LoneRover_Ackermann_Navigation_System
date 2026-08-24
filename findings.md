# Public navigation findings

这份文件只记录公开导航版本中仍然有效的工程结论，不保留比赛阶段日志、录包索引和已经放弃的实验方案。

## 定位

- 连续里程计由轮速与 IMU 的松耦合 EKF 提供；
- 轮速比例、转向比例、IMU 零偏和时间戳比增加传感器数量更影响最终效果；
- `odom -> base_link` 只由 EKF 维护，地图侧只提供静态地图关系或起始位姿；
- 调参顺序是先确认原始反馈，再确认 EKF 输出，最后检查地图和规划侧。

## 地图分辨率

1 cm 地图已经能表达车辆 footprint、雷达噪声和通道边界。5 mm、1 mm 虽然更细，但会增加地图、代价地图和规划器开销，对实际定位误差的改善有限，因此公开配置以 1 cm 为主。

## 全局规划、路径和控制器

`SmacPlannerHybrid` 本身就是 Nav2 的 Hybrid-A* 全局规划器，负责在最小转弯半径和车辆 footprint 约束下搜索可行路径。路径是规划结果，RPP 是路径跟踪控制器，两者职责不同。

窄通道中，连续导航点会带来较多规划和状态交接问题。Tube 把通道几何固化为中心、内侧、外侧三组路径，再由 RPP 跟踪，在线只做候选选择和安全复核。

## 倒车入口

倒车入口使用离线 LUT。LUT 以真实车辆参数逐个位姿生成候选，检查曲率、footprint、地图障碍和入口连接；运行时查表缩小搜索范围，再用当前代价地图复核。这样在线搜索量可控，入口方向也更稳定。

## 虚实结合调参

虚拟 Ackermann 车模加入了转向响应延迟、速度响应延迟和限幅，用来比较 Tube、RPP、速度和切换余量。离线结果用于缩小实体车测试范围，最终参数仍需在真实车辆上确认。

## 雷达边界

公开导航包只约定外部雷达驱动提供 `/scan_raw`。滤波、去躁、锥桶聚类和代价地图图层属于本仓库；具体设备驱动属于外部适配层，不在公开版本中。

## 2026-08-24 结构审计

- `src/mycar/mycar_navigation/` 是独立的旧式 `local_navigator_node` 包，只在包内部互相引用；当前比赛入口和 Nav2 链路没有引用它；
- `src/hobot_navigation/lidar_local_planner/` 同样只在包内部引用，当前雷达启动链路使用 Nav2 MPPI/RPP，没有启动这个旧式反应式节点；
- 当前速度链路由 `adaptive_speed_limiter` 和 `hobot_nav` 的 `ackermann_command_limiter` 组成，根 README 不应再把速度逻辑归到已移除的 `mycar_navigation`；
- `tools/vehicle_model/ackermann_vehicle_model.py` 提供的是 Ackermann 车模、舵机查表和几何换算函数，不是只能用于某一种下位机的标定程序；
- `shadow` 在工具名中是控制工程术语，但公开仓库入口改成 `virtual_vehicle` 更直观；`reverse_gate` 统一改成 `reverse_entry`，与比赛文档中的“倒车入口”一致；
- 工具按 `vehicle_model/`、`reverse_entry/`、`channel_tuning/`、`calibration/` 和 `offline_analysis/` 分组；Tube 共用的地图、footprint 和哈希函数放在 `channel_tuning/channel_asset_common.py`；重命名只涉及公开入口、导入和文档引用，不改变算法和输出格式。
