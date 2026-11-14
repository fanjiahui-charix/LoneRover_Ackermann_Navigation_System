# hobot_nav

`hobot_nav` is the public navigation integration package. It connects the
user-maintained base driver and EKF with Nav2, 2D lidar perception, costmaps,
Ackermann command limiting, static maps and offline Tube/LUT assets.

The competition task coordinator is implemented in
[`CompetitionSupervisor`](src/competition_supervisor.cpp), with Nav2 Action
handover handled by [`MotionActionArbiter`](src/motion_action_arbiter.cpp).
See [README_COMPETITION_EN.md](README_COMPETITION_EN.md) for its build option,
vehicle dependencies and complete deployment command.

## Runtime chain

```text
external LaserScan driver -> scan filter -> cone clustering
                                      -> cone costmap layer
wheel odometry + IMU -> ekf_fusion -> odom -> base_link
static map -> Nav2 global/local costmaps
                         -> SmacPlannerHybrid -> RPP / another controller
                         -> velocity smoother -> Ackermann limiter -> chassis
```

The public launch assumes an existing `/scan_raw` publisher and an existing
`base_link -> laser_link` static transform. The bundled base driver supplies the sensor static transforms when enabled.

## Start the public stack

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch hobot_nav navigation_core.launch.py \
  launch_base:=true \
  launch_lidar_pipeline:=true \
  launch_static_map:=true \
  enable_lidar_odom:=true
```

If the lidar driver publishes another topic, remap the perception pipeline:

```bash
ros2 launch hobot_nav lidar_pipeline.launch.py \
  raw_scan_topic:=/my_lidar/scan \
  filtered_scan_topic:=/scan
```

The command starts navigation and static-map playback only. It does not create
a map. The base driver parameters, IMU/odometry calibration and the map path
must be changed for another vehicle or track.

## What is included

- wheel odometry + IMU EKF integration and static vehicle TF;
- Smac Hybrid-A* global planning configuration;
- Ackermann-friendly Nav2 behavior trees without in-place spin recovery;
- 2D lidar filtering, deskew, cone clustering and costmap plugins;
- velocity smoothing and final Ackermann curvature/speed limiting;
- 1 cm map, reverse-entry LUTs, channel feasibility assets and three Tube
  choices in each direction;
- waypoint auditing, shadow-vehicle tuning and native replay helpers.

For ordinary navigation, a caller can submit `NavigateToPose` or
`NavigateThroughPoses` goals to `navigation_core.launch.py`. The competition
launch uses the Supervisor to coordinate its QR, reverse-entry, channel and
return stages.

## Coordinate and interface contract

The normal interfaces are:

| Interface | Meaning |
| --- | --- |
| `/odom/data_raw` | raw wheel odometry from the base driver |
| `/imu/data_raw` | raw IMU data from the base driver |
| `/odom` | EKF output used by Nav2 and deskew |
| `/scan_raw` | external lidar driver input |
| `/scan` | filtered scan used by costmaps and perception |
| `/cones/points` | clustered cone centers |
| `/map` | prebuilt static map |
| `/cmd_vel_nav` | velocity-smoother output |
| `/cmd_vel_safe` | final constrained command to the chassis |

The dynamic `odom -> base_link` transform belongs to `ekf_fusion`. The static
`base_link -> imu_link` and `base_link -> laser_link` transforms must have one
owner and match the actual installation.

## Build dependencies

ROS 2 Humble and Nav2 are external dependencies. The repository does not vendor
the robot platform SDK, image stack, lidar driver or Nav2 source. See the root
[`docs/DEPENDENCIES.md`](../../../docs/DEPENDENCIES.md) for the official upstream
links and the platform-specific build notes.

## Related documentation

- [`README_NAV_TUNING.md`](README_NAV_TUNING.md): virtual vehicle and offline
  tuning workflow;
- [`README_CHANNEL_MODES.md`](README_CHANNEL_MODES.md): Tube, LUT and local
  tracking assets;
- [`config/`](config/): Nav2, vehicle and costmap parameters;
- [`runtime/`](runtime/): compact generated assets;
- [`tools/README.md`](../../../tools/README.md): offline navigation utilities.
