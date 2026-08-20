# External dependencies

This repository contains the navigation algorithms and integration written for
the project. Platform packages are intentionally referenced as external
dependencies rather than copied into the source tree.

| Dependency | Role | Source |
| --- | --- | --- |
| ROS 2 Humble | middleware and message/runtime base | [ROS 2 documentation](https://docs.ros.org/en/humble/) |
| Nav2 | map server, costmaps, Smac, controllers, BT navigator and lifecycle | [navigation2](https://github.com/ros-navigation/navigation2) |
| D-Robotics platform configuration | only needed for an X5 cross-build/sysroot | [robot_dev_config](https://github.com/D-Robotics/robot_dev_config) |
| D-Robotics runtime packages | supplied by the target image when required by a vehicle deployment | [D-Robotics organization](https://github.com/D-Robotics/) |

The public navigation core does not require the vendor camera, image-memory,
QR, VLM, YOLO or BEV packages. It also does not select a particular lidar
driver. Provide a ROS 2 driver that publishes a valid `LaserScan` on
`/scan_raw`, or remap `lidar_pipeline.launch.py` to your own topic.

## Host build

For a normal ROS 2 host build, source the installed ROS environment and use
`colcon build`. The repository does not vendor Nav2 or a platform SDK.

## X5 cross-build

`x5_build.sh` is a helper for users who already have the target sysroot and the
external D-Robotics toolchain checkout. It does not download or copy that
checkout. Set `X5_TOOLCHAIN_FILE` and, if needed, `X5_CLEAR_COLCON_IGNORE`.

The X5 build validates the navigation source only. A successful host or
cross-build does not provide a chassis protocol, lidar firmware driver, or the
private competition task adapter.
