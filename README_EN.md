# LoneRover Ackermann Navigation

A ROS 2 navigation system for an Ackermann vehicle. The public repository focuses on localization, lidar perception, maps and costmaps, path planning, local tracking, speed constraints, and simulation-assisted offline tuning.

`LoneRover` has a direct meaning: `Lone` refers to a navigation stack built and refined independently by one developer, while `Rover` refers to the vehicle carrying the algorithms. It is the project name, not a third-party framework.

[中文 README](README.md)

## Public scope

This repository is the public navigation release. The task layer, vision pipeline, model files, competition-only entry points, and low-level device drivers with unclear provenance are outside the repository. The public code connects to external chassis and lidar drivers through standard ROS 2 interfaces, so third-party code and private competition dependencies are not presented as directly reproducible components.

The retained parts are the navigation components that can be studied, tested, and reused independently:

- wheel-odometry and IMU EKF inertial navigation;
- filtering and denoising of external `LaserScan` data, cone clustering, and Nav2 costmap layers;
- a 1 cm static map, static obstacles, dynamic obstacles, cone layer, keepout layer, and inflation layer;
- `SmacPlannerHybrid` for global planning. It is Nav2's Hybrid-A* planner for Ackermann vehicles;
- Nav2 path tracking in ordinary areas and Tube paths with RPP tracking in narrow channels;
- reverse-entry LUTs built with the real vehicle model, a delayed-response virtual vehicle, and offline evaluation tools;
- velocity smoothing, curvature limits, lateral-acceleration limits, reverse limits, and safe command output.

## System structure

```text
chassis serial / wheel odometry / IMU
                 │
                 ▼
origincarpro_base ──► ekf_fusion ──► odom -> base_link
                 │                         │
                 └──► external static TF  └──► Nav2 in the map frame

external lidar driver ──► /scan_raw ──► filtering / denoising ──► /scan
                                               │
                                               └──► cone clustering ──► cone costmap layer

static map ─────────────────────────────────────────────────────► global costmap
static map + /scan + cone layer + keepout layer ───────────────► local costmap

task layer provides a goal or a path
                 │
                 ▼
SmacPlannerHybrid ──► path ──► RPP / another Nav2 controller
                                               │
                                               ▼
                                  velocity smoothing and Ackermann limits
                                               │
                                               ▼
                                         /cmd_vel_safe
```

The task layer selects goals, routes, and the Tube or LUT to use. The public repository keeps the navigation execution side. Nav2's behavior tree handles planning, tracking, replanning, and recovery inside a navigation action; private competition decisions are not part of this package.

## Core techniques

### EKF inertial localization

`ekf_fusion` fuses chassis wheel feedback and IMU measurements with a loosely coupled EKF. In a loosely coupled design, wheel odometry and IMU data first become standard observations and are then used by one EKF update; each sensor driver, message format, and error model remains relatively independent.

The important part is calibration against the real vehicle: wheel scale, steering scale, IMU bias, timestamps, body frames, and turn-dependent observation noise must match the hardware. The filter publishes the dynamic `odom -> base_link` transform. The map relationship is supplied by the map or initial-pose side, which keeps one clear owner for the dynamic odometry transform.

The public configuration keeps normal and conservative parameter entry points. This makes it possible to validate raw feedback first and then adjust process and observation noise. Recording-only workflows are not included; offline tuning tools operate on ROS 2 data supplied by the user.

### Maps and costmaps

The course is a fixed environment, so the public release uses an existing 1 cm static occupancy map and does not include a mapping launch flow. The map provides one shared coordinate system for navigation, planning, and obstacle geometry.

The project also tested 5 mm and 1 mm grids. Finer resolution increases the cost of the map, costmaps, planning, and visualization, while vehicle dimensions, lidar noise, and localization error do not become smaller. For this vehicle, 1 cm is the more useful balance between precision, real-time performance, and maintenance.

The global costmap uses the static map and required obstacle information. The local costmap adds lidar obstacles, cone geometry, keepout regions, and inflation in real time. The cone layer consumes clustered geometry points and has no camera or model dependency.

### Global planning and local tracking

Ordinary areas and reverse-entry connections use `SmacPlannerHybrid`. Smac and Hybrid-A* are not two separate planners here: `SmacPlannerHybrid` is Nav2's Hybrid-A* implementation, generating feasible paths under the vehicle's minimum turning radius, direction, and footprint constraints.

The planner outputs a `Path`; a local controller tracks it and produces velocity commands. The public default is RPP. In narrow channels, the main idea is to track pre-validated Tube paths instead of repeatedly stacking navigation points. The responsibilities remain separate: a path describes where the vehicle should go, a planner searches or connects paths, and a controller tracks the selected path while handling local error.

### Tube channel paths

Many consecutive navigation points make global planning and state handoff fragile in narrow channels, and they do not provide a stable way to handle boundary obstacles. The public configuration keeps three lateral paths: `center`, `inner`, and `outer`, each available in clockwise and counter-clockwise directions. Runtime selection chooses a safe candidate, and RPP tracks it.

Tube paths have explicit geometry, simple switching logic, and a small online computational cost. Path generation and boundary checks are performed offline. Runtime work is limited to candidate selection, a current-costmap safety check, and safe output. This turns narrow-channel avoidance from a large collection of navigation points into a small set of verified path templates.

### Reverse-entry LUT and simulation-assisted tuning

The reverse-entry LUT is generated and checked with the actual Ackermann vehicle model rather than a few hand-written target points. Candidates are checked for minimum turning radius, vehicle footprint, map obstacles, and entry connectivity. Runtime looks up a small set of candidates from the current pose and performs a final check against the current costmap.

Offline tuning keeps two vehicle models: one uses the real vehicle planning configuration, and the other includes steering-response delay, speed-response delay, and limits. Tube, RPP, speed, and delay parameters can be compared in the virtual vehicle before a small number of real-vehicle checks. The related tools are in `tools/` and `tools/channel_tuning/`.

### Speed control and safe output

Speed control is not merely a planner parameter. The public code generates smooth speed from curvature, lateral-acceleration limits, forward and reverse limits, acceleration, and braking limits. The command side also checks Ackermann radius, reverse direction, and stale commands. The chassis receives only the restricted `/cmd_vel_safe` output.

## Repository layout

```text
src/hobot_navigation/
├── hobot_nav/              # Nav2 launch, parameters, costmaps, Tube, utilities
├── lidar_perception/        # LaserScan filtering, cone clustering, cone layer
├── lidar_local_planner/     # lidar local-planning helpers
└── lidar_web_viewer/        # lightweight viewer for standard lidar topics

src/mycar/
├── origincarpro_base/       # chassis serial, wheel odometry, IMU, and base TF
├── ekf_fusion/              # wheel + IMU EKF inertial navigation
├── simple_lidar_odom/       # external lidar-odometry interface and helpers
├── mycar_navigation/        # clustering, path, and speed-control helpers
└── origincar_description/   # delayed-response virtual Ackermann vehicle

tools/                       # LUT, Tube, RPP, and shadow-vehicle tools
docs/                        # dependencies, public scope, and tuning notes
```

## Requirements

- Ubuntu 22.04;
- ROS 2 Humble;
- Nav2 and its standard plugins;
- a chassis that provides wheel odometry, IMU, and an Ackermann command interface;
- a user-installed lidar driver that publishes `sensor_msgs/msg/LaserScan`;
- a static map consistent with the vehicle footprint, frames, and initial pose.

The lidar driver is intentionally not included. Its source is not fully certain and it belongs to the device-adaptation layer. Any driver that publishes standard `/scan_raw` can be used. X5 cross-compilation uses a toolchain prepared outside this repository; platform SDKs, system packages, and vendor build repositories are not copied into the public tree.

See [`docs/DEPENDENCIES.md`](docs/DEPENDENCIES.md) for dependencies and [`docs/OPEN_SOURCE_SCOPE.md`](docs/OPEN_SOURCE_SCOPE.md) for the boundary of the release.

## Build and run

With ROS 2 Humble and Nav2 installed:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ros_workspace_open_source
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
```

Launch the public navigation chain:

```bash
ros2 launch hobot_nav navigation_core.launch.py
```

This starts the static map, chassis interface, EKF, lidar processing, Nav2, and safe velocity output. If the external lidar driver uses another topic, remap it:

```bash
ros2 launch hobot_nav navigation_core.launch.py \
  raw_scan_topic:=/my_lidar/scan \
  enable_lidar_odom:=false
```

This launch is for navigation-component validation and does not include a complete competition task entry point. With a custom chassis, publish `/odom/data_raw` and `/imu/data_raw`, then subscribe to `/cmd_vel_safe`.

## Main topics and TF

| Direction | Interface | Description |
|---|---|---|
| input | `/odom/data_raw` | raw wheel odometry from the chassis |
| input | `/imu/data_raw` | chassis IMU |
| input | `/scan_raw` | raw LaserScan from the external lidar driver |
| output | `/odom` | EKF-fused odometry |
| output | `/scan` | filtered LaserScan |
| output | `/cones/points` | clustered cone geometry |
| output | `/map` | static OccupancyGrid |
| output | `/plan`, `/local_plan` | global and local paths |
| output | `/cmd_vel_nav` | Nav2 velocity command |
| output | `/cmd_vel_safe` | restricted chassis command |

The main TF chain is `map -> odom -> base_link`. Static sensor frames include `base_link -> imu_link` and `base_link -> laser_link`. Camera TF and vision topics are not part of the public runtime.

## Offline tools

- `tools/generate_reverse_gate_lut.py`: generate reverse-entry candidates;
- `tools/validate_reverse_gate_paths.py`: check curvature, footprint, and map feasibility;
- `tools/offline_mppi_shadow_sim.py`: evaluate tracking with a virtual vehicle for a supplied path;
- `tools/channel_tuning/`: generate and sweep Tube, connector, RPP, and cone-avoidance parameters;
- `hobot_nav/scripts/waypoint_audit.py`: check waypoint coordinates, headings, and spacing;
- `hobot_nav/scripts/nav_doctor.py`: check basic topics, TF, and parameters required by navigation.

The offline tools do not search for recorded data and do not depend on bags stored in the repository. Keep personal data outside the repository and pass it explicitly through command-line arguments.

## Debugging notes

Foxglove is useful for observing topic rates, TF, paths, and costmaps; RViz generally has more interaction overhead in large-map scenes. When localization behaves incorrectly, inspect raw wheel feedback, IMU data, EKF output, and the map-frame relationship separately. Vehicle spinning or path oscillation can come from localization error, curvature limits, the local controller, or speed limits; changing one waypoint alone is rarely a complete diagnosis.

For narrow-channel debugging, reduce consecutive navigation points and first validate Tube geometry, RPP lateral error, obstacle inflation, and switching margins. Screen parameters with the virtual vehicle and offline paths before real-vehicle validation.

## License and third-party dependencies

This repository is responsible only for the code and documentation in its public directories. ROS 2, Nav2, the chassis platform packages, and the lidar driver follow their own licenses and release mechanisms. Read [`docs/DEPENDENCIES.md`](docs/DEPENDENCIES.md) before use.
