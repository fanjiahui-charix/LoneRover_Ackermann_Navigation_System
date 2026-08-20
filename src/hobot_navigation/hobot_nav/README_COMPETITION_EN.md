# Competition runtime

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
~~~

The public file uses placeholders for the VLM address and admission digest.
The vehicle uses the values installed in its own runtime environment.

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
channel_v2_active admission files. Keep the vehicle stationary until all
readiness checks pass.

The full package description, parameter table, and file map are in
[README.md](README.md). Offline LUT, Tube, and shadow tools are indexed in
[tools/README.md](../../../tools/README.md).

