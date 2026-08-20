# Public release scope

This repository is the navigation-only open-source release of the project. It
contains the code and assets needed to study and build the navigation core:

- wheel odometry and IMU EKF integration;
- user-maintained Ackermann base interface and command limiting;
- external `LaserScan` processing, cone clustering and costmap layers;
- static-map Nav2 configuration, Smac Hybrid-A* planning and Ackermann BTs;
- RPP/local tracking interfaces, Tube paths, reverse-entry LUTs and offline
  shadow-vehicle tuning tools;
- the URDF/RViz vehicle model used for virtual and real-vehicle parameter work.

The following are intentionally excluded from this public tree:

- QR recognition source, purchased QR models and QR-specific messages;
- VLM/Ollama, sign recognition, YOLO, BEV, camera and zero-copy image code;
- the private competition Supervisor, task state machine and mission route;
- the source of the lidar serial driver where ownership is uncertain;
- D-Robotics/TogetheROS source copies, platform SDKs and `robot_dev_config`;
- rosbag data, vehicle logs, capture-only packages, generated build trees and
  external competition documents.

The public launch is therefore a reusable navigation entry point, not the
private full competition launch. A user must provide a compatible chassis,
static transforms, a lidar driver and any task-level goal source.

External dependencies and official upstream links are listed in
[`DEPENDENCIES.md`](DEPENDENCIES.md). Git history in the private workspace is
not part of the public source contract; only the cleaned source tree is meant
to be consumed here.
