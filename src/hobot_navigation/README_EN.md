# hobot_navigation

This directory groups the ROS 2 navigation packages for the Ackermann vehicle:

- `hobot_nav`: Nav2 parameters, static maps, behavior trees, Tube/LUT assets, costmap integration, and command limiting;
- `lidar_perception`: filtering, cone clustering, and costmap layers for an external 2D lidar;
- `lidar_local_planner`: a lightweight reactive lidar-planning reference;
- `lidar_web_viewer`: a low-load browser viewer for `LaserScan`;
- `adaptive_speed_limiter`: speed limiting from curvature, obstacle clearance, and goal distance.

The package-level navigation description is [`hobot_nav/README.md`](hobot_nav/README.md).
The Chinese entry point is [`hobot_nav/README_CN.md`](hobot_nav/README_CN.md).
