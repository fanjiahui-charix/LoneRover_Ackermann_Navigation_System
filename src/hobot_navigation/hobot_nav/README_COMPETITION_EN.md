# Competition runtime

## Build and vehicle dependencies

The mission implementation is [`src/competition_supervisor.cpp`](src/competition_supervisor.cpp), with Action handover managed by [`src/motion_action_arbiter.cpp`](src/motion_action_arbiter.cpp). The code and launch chain were recovered from [TarochLee/ros_workspace@aa018d6](https://github.com/TarochLee/ros_workspace/tree/aa018d6627b272c0797bab58faaa3dd07884f2d1).

```bash
colcon build --packages-up-to hobot_nav \
  --cmake-args -DBUILD_COMPETITION_SUPERVISOR=ON
source install/local_setup.bash
```

Install the vehicle's N10, camera, QR, VLM, HDMI and BEV ROS packages. Provide the camera calibration and BEV model/semantic map; the admitted channel assets are installed in `runtime/channel_v2/`. Package names and X5 build instructions are in [DEPENDENCIES.md](../../../docs/DEPENDENCIES.md). The field command uses the bundled channel asset path and digest. Set camera, BEV and VLM parameters for the vehicle deployment.

The current competition deployment uses one complete launch command:
competition_runtime.launch.py. The wrapper and diagnostic launch files are not
the competition procedure described here.

The runtime loads an existing 1 cm static map. It does not create a map during
the competition run.

## Field command

Stop the desktop display first:

~~~bash
systemctl stop lightdm
~~~

Then run this on the vehicle:

~~~bash
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
~~~

Set the VLM server address for the vehicle network. The channel admission digest matches the installed `runtime/channel_v2/runtime_admission.csv`.

## Runtime chain

~~~text
lower controller + wheel feedback + IMU
        -> origincarpro_base
        -> wheel odometry + IMU EKF
        -> map/odom fixed-start alignment
        -> existing 1 cm map_server map
        -> Nav2 costmaps and SmacPlannerHybrid
        -> MPPI in open areas
        -> Tube/RPP in the narrow channel
        -> velocity smoother and command limiter
        -> /cmd_vel_safe
        -> vehicle chassis
~~~

The same launch also starts the camera, QR pipeline, cone detector, Supervisor,
optional VLM sidecar, and the stage-gated BEV stack required by the current
vehicle runtime. Starting the BEV stack and accepting a BEV correction are
separate controls owned by the current Supervisor.

## What the runtime uses

- Existing map:
  maps/rdk_2026_hospital_static_1cm.yaml and its PGM file;
- Fixed start:
  waypoints/fixed_start_points.yaml;
- Localization:
  lower-controller wheel feedback + IMU through ekf_fusion;
- Global planning:
  Nav2 SmacPlannerHybrid;
- Open-area local planning:
  MPPI;
- Narrow-channel tracking:
  selected Tube asset + RPP;
- Mission control:
  competition_supervisor;
- Final command:
  velocity smoother, command limiter, and /cmd_vel_safe.

No mapping command belongs in the field procedure. Do not add a mapping node,
AMCL, or pose-graph localization to this launch chain.

## Before starting

Check the physical emergency stop, P pose, ROS domain, lower-controller
feedback, IMU, N10, camera, static TF, costmap freshness, and
the installed channel admission files. Keep the vehicle stationary until all
readiness checks pass.

The full package description, parameter table, and file map are in
[README.md](README.md). Offline LUT, Tube, and shadow tools are indexed in
[tools/README.md](../../../tools/README.md).

